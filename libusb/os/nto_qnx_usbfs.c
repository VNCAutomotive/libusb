
/*
 * QNX Neutrino backend for libusb
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "nto_qnx_usbfs.h"

#define	INIT_SLEEP_SECS	1

/**
 * USB control transfers must complete within 5 seconds.
 */
#define CONTROL_TRANSFER_TIMEOUT_MS 5000

/* Maximum number of buses possible (read-only connection is not
   compatible with dynamically connecting to devices, so you have to
   iterate through ALL possible busno/devno combinations */
#define MAX_BUSES 10
/* Maximum device number per bus */
#define MAX_DEVICES 64


#define dump_itransfer_info(itransfer) \
    do {                                                                \
        struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer); \
        usbi_dbg("Dumping itransfer data\n"                             \
                 "\nitransfer = %p\n"                                   \
                 "device_handle = %p\n"                                 \
                 "flags = 0x%x\n"                                       \
                 "endpoint = 0x%x\n"                                    \
                 "type = %d\n"                                          \
                 "timeout = %d\n"                                       \
                 "status = %d\n"                                        \
                 "length = %d\n"                                        \
                 "actual_length = %d\n"                                 \
                 "callback = %p\n"                                      \
                 "user_data = %p\n"                                     \
                 "buffer = %p\n",                                       \
                 itransfer,                                             \
                 transfer->dev_handle,                                  \
                 transfer->flags,                                       \
                 transfer->endpoint,                                    \
                 transfer->type,                                        \
                 transfer->timeout,                                     \
                 transfer->status,                                      \
                 transfer->length,                                      \
                 transfer->actual_length,                               \
                 transfer->callback,                                    \
                 transfer->user_data,                                   \
                 transfer->buffer                                       \
                 );                                                     \
    } while(0)



/* urb_q structure, used for the internal polling thread which checks
   on the status of urbs */
struct urb_tailq {
    TAILQ_ENTRY(urb_tailq) chain;
    /* struct usbd_urb * urb; */
    struct usbi_transfer * itransfer;
};

/* Global list of urb's which need to be polled by the internal thread

   This is the head of the urb_tailq */
TAILQ_HEAD(urb_head_type, urb_tailq) 
urb_head;

/* Channel id's for thread communication */
static int urb_thread_chid;
static int urb_main_coid;

/* The message type for communicating with the thread, it only accepts
   pulses at the moment */
typedef union thread_wakeup thread_wakeup_t;
union thread_wakeup
{
    struct _pulse pulse;
};

/* Forward function declarations */
static void qnx_async_io_callback(struct usbd_urb * urb, struct usbd_pipe * upipe, void * userdata);

/**
 * This mutex is used to stop multiple calls to op_init() from doing
 * anything except on the first call.
 */
static pthread_mutex_t init_mutex = PTHREAD_MUTEX_INITIALIZER;

/* TODO: This variable indicates the level of debugging. It's set by
   ctx->debug in the init, should probably be removed or replaced with
   something more consistent with libusb. */
static int usb_debug = 2;

/* Keeping these two global connections is gross, it does not appear
   that there is a better way at this time. */

/* Normal READ WRITE connection */
struct usbd_connection * global_connection;

/* READ ONLY connection */
struct usbd_connection * global_ro_connection;


/* Return an nto_qnx_device_priv structure from the libusb device structure */
static struct nto_qnx_device_priv * __device_priv(struct libusb_device *dev)
{
    return (struct nto_qnx_device_priv *) dev->os_priv;
}

/* Return an nto_qnx_device_handle_priv structure from the libusb device structure */
static struct nto_qnx_device_handle_priv *__device_handle_priv(
	struct libusb_device_handle *handle)
{
	return (struct nto_qnx_device_handle_priv *) handle->os_priv;
}

/* Return a libusb_device from a previous session */
static struct libusb_device * get_device_from_session(struct libusb_context *ctx,
                                                      int busnum, int devaddr)
{
    unsigned long session_id = busnum << 8 | devaddr;
    return usbi_get_device_by_session_id(ctx, session_id);
}

/* Using this is the preferred method of translating errno error codes
   to libusb error codes, as well as converting usbd errors to libusb
   errors. Keeping the translation here keeps things clean. */
static int qnx_err_to_libusb(int error) 
{
    switch (error) 
    {
    case EOK:
        return LIBUSB_SUCCESS;
    case EMSGSIZE:
        return LIBUSB_ERROR_OTHER;
    case ENOMEM:
        return LIBUSB_ERROR_NO_MEM;
    case ENODEV:
        return LIBUSB_ERROR_NO_DEVICE;
    case EACCES:
        return LIBUSB_ERROR_ACCESS;
    case EAGAIN:
    case EBUSY:
        return LIBUSB_ERROR_BUSY;
    /* case USBD_STATUS_STALL: */
    /*     return LIBUSB_ERROR_PIPE; */
    default:
        return LIBUSB_ERROR_OTHER;
    }
}

static int qnx_transfer_status(struct usbi_transfer *itransfer, _uint32 ustatus)
{
    _uint32 urb_masked = USBD_URB_STATUS_MASK & ustatus & ~USBD_STATUS_CMP_ERR;
    _uint32 usb_masked = USBD_USB_STATUS_MASK & ustatus & ~USBD_STATUS_CMP_ERR;
    _uint32 urb_err = USBD_URB_STATUS_MASK & ustatus &  USBD_STATUS_CMP_ERR;
    _uint32 usb_err = USBD_USB_STATUS_MASK & ustatus &  USBD_STATUS_CMP_ERR;

    switch (urb_masked)
    {
    case USBD_STATUS_CMP:
        return LIBUSB_TRANSFER_COMPLETED;
    case USBD_STATUS_ABORTED:
        return LIBUSB_TRANSFER_CANCELLED;
    case USBD_STATUS_TIMEOUT:
        return LIBUSB_TRANSFER_TIMED_OUT;
    default:
        usbi_err (ITRANSFER_CTX (itransfer), "transfer error: unknown.");
        return LIBUSB_TRANSFER_ERROR;
    }

    switch (usb_masked)
    {
    case USBD_STATUS_STALL:
        usbi_warn (ITRANSFER_CTX(itransfer), "transfer warning: pipe is stalled.");
        return LIBUSB_TRANSFER_STALL;
    case USBD_STATUS_DATA_OVERRUN:
        usbi_err (ITRANSFER_CTX (itransfer), "transfer error: data overrun.");
        return LIBUSB_TRANSFER_OVERFLOW;
    case USBD_STATUS_BUFFER_OVERRUN:
        usbi_err (ITRANSFER_CTX (itransfer), "transfer error: buffer overrun.");
        return LIBUSB_TRANSFER_OVERFLOW;
    default:
        usbi_err (ITRANSFER_CTX (itransfer), "transfer error: unknown.");
        return LIBUSB_TRANSFER_ERROR;
    }

    if (urb_err || usb_err)
    {
        // we fall here if status is simply USBD_STATUS_CMP_ERR and no specific error code available.
        usbi_err (ITRANSFER_CTX (itransfer), "transfer cmp error: urb_err=%x, usb_err=%x", urb_err, usb_err);
        return LIBUSB_TRANSFER_ERROR;
    } else {
        // this case can't be reachable now, but better to be kept in case
        usbi_err (ITRANSFER_CTX (itransfer), "qnx_transfer_status() invalid log");
        return LIBUSB_TRANSFER_ERROR;
    }
}


/* Check a urb and itransfer to see if they are complete, if they are:
   send a message down the pipe() so that the application using this
   library knows to call handle_events. */
static int handle_urb_status(struct urb_tailq * node)
{

    int r = 0;
    _uint32 ustatus;
    _uint32 usize;
    int status;

    struct usbi_transfer *itransfer = node->itransfer;
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);

    status = usbd_urb_status(tpriv->urb, &ustatus, &usize);

    if (status == EBUSY)
    {
        /* then we are done here, keep polling */
    } else {

        /* struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer); */
        struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(transfer->dev_handle);
        uint32_t message;

        /* handle ustatus */
        int ustatus_masked = (ustatus & USBD_URB_STATUS_MASK);
        
        usbi_dbg ("ustatus_masked: %d, %s", ustatus_masked, strerror(status));
        
        switch (ustatus_masked)
        {
        case USBD_STATUS_INPROG:
            usbi_info(ITRANSFER_CTX(itransfer), "USBD_STATUS_INPROG found!");
            /* then we are done here, keep polling */
            goto handle_urb_status_skip;
            break;
        case USBD_STATUS_CMP:
            usbi_info(ITRANSFER_CTX(itransfer), "USBD_STATUS_CMP found!");
            usbi_info(ITRANSFER_CTX(itransfer), "transfer complete...");
            usbi_info (ITRANSFER_CTX (itransfer), "an async io operation has completed");
            message = MESSAGE_ASYNC_IO_COMPLETE;
            break;
        case USBD_STATUS_CMP_ERR:
            usbi_info(ITRANSFER_CTX(itransfer), "USBD_STATUS_CMP_ERR found!");
            message = MESSAGE_ASYNC_IO_COMPLETE;
            /* TODO: Handle other completion related errors */
            break;
        case USBD_STATUS_TIMEOUT:
            usbi_info(ITRANSFER_CTX(itransfer), "USBD_STATUS_TIMEOUT found!");
            /* TODO: handle timeout */
            /* Here we should cancel */
            TAILQ_REMOVE(&urb_head, node, chain);
            r = 1;
            goto handle_urb_status_skip;
            break;
        case USBD_STATUS_ABORTED:
            usbi_info(ITRANSFER_CTX(itransfer), "USBD_STATUS_ABORTED found!");
            message = MESSAGE_ASYNC_IO_COMPLETE;
            /* TODO: handle aborted */
            break;
        }
        
        write (hpriv->fds[1], &message, sizeof (message));
        write (hpriv->fds[1], &itransfer, sizeof (itransfer));
        TAILQ_REMOVE(&urb_head, node, chain);
        r = 1;
    handle_urb_status_skip:
        return r;
    }
    return r;
}

/* Simple thread that polls a given linked list of urbs to check if
   any of them need to have their status reported through the fds.

   It's very possible that this internal thread based approach is not
   necessary. There seems to be nothing that isn't either synchronous
   or which calls their callback function.

*/
static void *urb_polling_thread_main(void *arg0)
{
    thread_wakeup_t msg;
    int rcvid;

    while(1)
    {
        struct urb_tailq * node;
        struct urb_tailq * delete_node = NULL;

        if (TAILQ_EMPTY(&urb_head))
        {
            while (1)
            {
                usbi_dbg("thread sleeping...");
                rcvid = MsgReceive(urb_thread_chid, &msg, sizeof(msg), NULL);
                if (rcvid == 0)
                {
                    /* message received was a pulse, wakeup thread */
                    usbi_dbg ("pulse code received: %d, thread woken", (int)msg.pulse.code);
                    break;
                } 
            }
            
        } else {
            if (!TAILQ_EMPTY(&urb_head))
            {
                TAILQ_FOREACH(node, &urb_head, chain)
                {
                    int status;

                    /* Delete node only if handle_urb_status returns
                       true (1) this prevents a segfault from
                       accessing freed memory. Performing it in this
                       specific order also avoids a race condition. */
                    if (delete_node) free(delete_node);

                    status = handle_urb_status(node);

                    if (status) delete_node = node;
                    
                }
            }
        }

        struct timespec stime;
        stime.tv_sec = 0;
        stime.tv_nsec = 100000;

        nanosleep(&stime, NULL);           /* sleep 100 milliseconds */
    }
    return NULL;
}


/* libusb doesn't support dynamic insertion so neither do we */
static void device_insertion_callback(struct usbd_connection * connection,
                                      usbd_device_instance_t * instance)
{
    usbi_dbg ("device_insertion_callback: starting");

    return;
}

/* We don't DIRECTLY support dynamic removal, but there are routines
   which will be called when a device removal is detected. In order to
   conform with libusb we do it in routines other than this one. */
static void device_removal_callback(struct usbd_connection * connection,
                                    usbd_device_instance_t * instance)
{
    usbi_dbg ("device_removal_callback: starting");

    uint32_t message = MESSAGE_DEVICE_GONE;
    libusb_context *ctx = NULL; /* NULL so that we can get the default context */
    struct libusb_device_handle *handle;
    struct libusb_device *device;
    struct nto_qnx_device_handle_priv *hpriv;

    int busno, devno;
    busno = instance->path, devno = instance->devno;

    /* Get the default context */
    USBI_GET_CONTEXT(ctx);
    if(ctx == NULL)
    {
        usbi_dbg ("device_removal_callback: context NA");
        return;
    }
    pthread_mutex_lock(&ctx->open_devs_lock);
    list_for_each_entry(handle, &ctx->open_devs, list, libusb_device_handle) {
        device = handle->dev;
        if ( busno == device->bus_number
             && devno == device->device_address)
        {
            usbi_dbg("Sending notification to libusb via pipe that this "
                     "device has been disconnected from the io-usb stack");
            hpriv = __device_handle_priv(handle);
            write (hpriv->fds[1], &message, sizeof (message));
        }
    }
    pthread_mutex_unlock(&ctx->open_devs_lock);

    return;
}


/* Perform initialization of your backend. You might use this function
 * to determine specific capabilities of the system, allocate required
 * data structures for later, etc.
 *
 * This function is called when a libusb user initializes the library
 * prior to use.
 *
 * Return 0 on success, or a LIBUSB_ERROR code on failure.
 */
static int op_init(struct libusb_context * ctx)
{
    /* the functions included here do nothing, they are to indicate to
       usbd_connect that we wish for a read/write connection */
    usbd_funcs_t funcs = { _USBDI_NFUNCS,
                           device_insertion_callback,
                           device_removal_callback,
                           0 };
    usbd_device_ident_t interest = { USBD_CONNECT_WILDCARD, /* vendor */
                                     USBD_CONNECT_WILDCARD, /* device */
                                     USBD_CONNECT_WILDCARD, /* class */
                                     USBD_CONNECT_WILDCARD, /* subclass */
                                     USBD_CONNECT_WILDCARD  /* protocol */ };

    usbd_connect_parm_t parm = { 0, USB_VERSION, USBD_VERSION, 0, 0, 0, 0, &interest, &funcs, 0 };

    /* Adding null for callbacks allows creation of a connection in read-only mode: */
    usbd_connect_parm_t ro_parm = { 0, USB_VERSION, USBD_VERSION, 0, 0, 0, 0, &interest, NULL, 0 }; 

    int status;
    
    usb_debug = 0; /* 3; ctx->debug; */

    if (pthread_mutex_trylock(&init_mutex) != 0) {
        usbi_warn(ctx, "already initialised");
        return LIBUSB_ERROR_BUSY;
    }

    /* Connect with a read only connection, this allows interacting in
       a read-only manner with devices which are already in use by
       something else. This type of behaviour is required by libusb
       but I cannot figure out a better way to implement it without
       making changes to libusbdi */
    if ((status = usbd_connect(&ro_parm, &global_ro_connection)) != EOK) {
        usbi_err(ctx, "usdb_connect() for read only failed");
        pthread_mutex_unlock(&init_mutex);
        return qnx_err_to_libusb(status);
    }

    /* Connect with a read/write connection, this lets us interact
       with usb devices in read/write mode */
    if ((status = usbd_connect(&parm, &global_connection)) != EOK) {
        usbi_err(ctx, "usdb_connect() failed");
        pthread_mutex_unlock(&init_mutex);
        return qnx_err_to_libusb(status);
    }

    /* sleep a while to let the USB stack send us insertion notifications */
    if (usb_debug >= 2) {
        usbi_info(ctx, "sleeping currently");
    }


    /* Initialize urb list */
    TAILQ_INIT(&urb_head);

    urb_thread_chid = ChannelCreate(0);
    urb_main_coid = ConnectAttach(0,0,
                                  urb_thread_chid,
                                  _NTO_SIDE_CHANNEL,
                                  0);

    pthread_create(NULL, NULL, urb_polling_thread_main, NULL);

    sleep(INIT_SLEEP_SECS);

    return 0;
}

/* Helper function to read the configuration specified by `config' on
   the device `dev' into the buffer `*buffer'. Performs allocation
   using malloc on the buffer pointer `buffer' */
static int read_configuration(unsigned char **buffer, struct libusb_device *dev, int config)
{
    usbi_dbg("reading configuration %d", config);
    int r;
    unsigned char tmp[8];

    struct libusb_config_descriptor config_descriptor;
    struct nto_qnx_device_priv *dpriv = __device_priv(dev);
    struct usbd_device *usbd_d = dpriv->usbd_device;
    
    usbi_dbg("trying to read the first few bytes of the configuration descriptor");
    r = usbd_descriptor(usbd_d, 0, USB_DESC_CONFIGURATION,
                        USB_RECIPIENT_DEVICE, config, 0,
                        tmp, sizeof tmp);
    if (r == EMSGSIZE)
    {
      /* Expected as only attemping a short read */
      r = 0;
    }

    if (r != 0)
    {
      usbi_dbg("Failed to read start of configuration descriptor, %d", r);
      return LIBUSB_ERROR_IO;
    }

    usbi_dbg("parsing the descriptor");
    usbi_parse_descriptor(tmp, "bbw", &config_descriptor, 0);

    usbi_dbg("mallocing the required memory in order to store the full descriptor");
    *buffer = (unsigned char *) malloc(config_descriptor.wTotalLength);
    if (!(*buffer))
    {
        return LIBUSB_ERROR_NO_MEM;
    }

    usbi_dbg("reading the full descriptor and dumping it into the memory we allocated");
    r = usbd_descriptor(usbd_d, 0, USB_DESC_CONFIGURATION,
                        USB_RECIPIENT_DEVICE, config, 0,
                        *buffer, config_descriptor.wTotalLength);

    if (r != 0)
    {
      usbi_dbg("Failed to read full configuration descriptor, %d", r);
      return LIBUSB_ERROR_IO;
    }

    usbi_dbg("finished reading configuration %d", config);
    return r;
}

/* Initialize the data of the device `dev'. This includes the
   initialization of some of the os-private data contained in
   `dev'. */
static int initialize_device(struct libusb_device *dev, uint8_t busnum, uint8_t devaddr, 
                             struct usbd_device * usbd_d)
{
    /* === libusb === */
    struct nto_qnx_device_priv * priv = __device_priv(dev);

    /* === internal === */
    int r = 0;
    int status;
    unsigned char * device_buffer = malloc(DEVICE_DESC_LENGTH);
    if (!device_buffer)
    {
      usbi_dbg("Failed to allocate memory for device buffer");
      return LIBUSB_ERROR_NO_MEM;
    }

    dev->bus_number = busnum;
    dev->device_address = devaddr;

    /* storing the usbd device as required by the read_configuration
       function*/
    priv->usbd_device = usbd_d;

    status = usbd_descriptor(usbd_d, 0 /* GET */, USB_DESC_DEVICE,
                             USB_RECIPIENT_DEVICE, 0, 0,
                             device_buffer, DEVICE_DESC_LENGTH);

    /* Cache the device descriptor as required by libusb */
    priv->dev_descriptor = device_buffer;

    /* TODO: This should work, number of configurations is the last
       byte in the device descriptor, could be cleaner. */
    dev->num_configurations = device_buffer[DEVICE_DESC_LENGTH - 1];

    /* TODO: handle error conditions as well as alternate configurations */

    /* We are going to guess that current configuration used is the
       first (configuration index 0) but we are also going to indicate that
       we aren't sure by putting -1 in selected configuration. When we
       ask later for the active configuration with the
       op_get_configuration function it will check for -1 and use a
       control packet to get the configuration if we aren't sure */
    priv->selected_configuration = -1;
    r = read_configuration(&priv->config_descriptor, dev, 0);
    
    /* priv->config_descriptor = config_buffer; */


    priv->connection = global_ro_connection;
    TAILQ_INIT(&priv->claimed_interfaces);

    return r;
}

/* Enumerate the device specified by it's bus number `busnum' and its
   device address `devaddr'. The current libusb context `ctx' is first
   checked to see if this device has been enumerated by the current
   session. */
static int enumerate_device(struct libusb_context *ctx, struct discovered_devs **_discdevs,
                            uint8_t busnum, uint8_t devaddr, struct usbd_device * usbd_d)
{
    /* === libusb === */
    struct discovered_devs *discdevs = *_discdevs;
    struct libusb_device *dev;
    unsigned long session_id;
    int need_unref = 0;
    
    /* === internal === */
    int r = 0;

    /* This session id should be relatively unique 
       
       TODO: is there an actual assigned session id I can
       use?*/
    session_id = busnum << 8 | devaddr;
    
    usbi_dbg("busnum %d devaddr %d session_id %ld", busnum, devaddr, 
             session_id);
    
    dev = usbi_get_device_by_session_id(ctx, session_id);
    if (dev)
    {
        usbi_dbg("using existing device for %d/%d (session %ld), dev = %p",
                 busnum, devaddr, session_id, dev);
    }
    else
    {
        usbi_dbg("allocating new device for %d/%d (session %ld)",
                 busnum, devaddr, session_id);
        
        dev = usbi_alloc_device(ctx, session_id);
        if (!dev) return LIBUSB_ERROR_NO_MEM;
        need_unref = 1;
        
        /* initialize device */
        r = initialize_device(dev, busnum, devaddr, usbd_d);

        if (r < 0)
            goto out;
        r = usbi_sanitize_device(dev);
        if (r < 0)
            goto out;
    }

    discdevs = discovered_devs_append(*_discdevs, dev);
    if (!discdevs) 
        r = LIBUSB_ERROR_NO_MEM;
    else 
        *_discdevs = discdevs;

 out:
	if (need_unref)
        libusb_unref_device(dev);
	return r;
}

/* Enumerate all the USB devices on the system, returning them in a list
 * of discovered devices.
 *
 * Your implementation should enumerate all devices on the system,
 * regardless of whether they have been seen before or not.
 *
 * When you have found a device, compute a session ID for it. The session
 * ID should uniquely represent that particular device for that particular
 * connection session since boot (i.e. if you disconnect and reconnect a
 * device immediately after, it should be assigned a different session ID).
 * If your OS cannot provide a unique session ID as described above,
 * presenting a session ID of (bus_number << 8 | device_address) should
 * be sufficient. Bus numbers and device addresses wrap and get reused,
 * but that is an unlikely case.
 *
 * After computing a session ID for a device, call
 * usbi_get_device_by_session_id(). This function checks if libusb already
 * knows about the device, and if so, it provides you with a libusb_device
 * structure for it.
 *
 * If usbi_get_device_by_session_id() returns NULL, it is time to allocate
 * a new device structure for the device. Call usbi_alloc_device() to
 * obtain a new libusb_device structure with reference count 1. Populate
 * the bus_number and device_address attributes of the new device, and
 * perform any other internal backend initialization you need to do. At
 * this point, you should be ready to provide device descriptors and so
 * on through the get_*_descriptor functions. Finally, call
 * usbi_sanitize_device() to perform some final sanity checks on the
 * device. Assuming all of the above succeeded, we can now continue.
 * If any of the above failed, remember to unreference the device that
 * was returned by usbi_alloc_device().
 *
 * At this stage we have a populated libusb_device structure (either one
 * that was found earlier, or one that we have just allocated and
 * populated). This can now be added to the discovered devices list
 * using discovered_devs_append(). Note that discovered_devs_append()
 * may reallocate the list, returning a new location for it, and also
 * note that reallocation can fail. Your backend should handle these
 * error conditions appropriately.
 *
 * This function should not generate any bus I/O and should not block.
 * If I/O is required (e.g. reading the active configuration value), it is
 * OK to ignore these suggestions :)
 *
 * This function is executed when the user wishes to retrieve a list
 * of USB devices connected to the system.
 *
 * Return 0 on success, or a LIBUSB_ERROR code on failure.
 */

static int op_get_device_list(struct libusb_context *ctx,
                              struct discovered_devs **_discdevs)
{

    /* === libusb === */
    struct discovered_devs *discdevs = *_discdevs;

    /* === internal === */
    /* enumerate devices, adding session ids if needed */
    int r = 0;
    int status = 0;
    int busno, devno;

    usbd_device_instance_t instance; /* manually configured instance */
    struct usbd_device * usbd_d;

    for (busno = 0; busno < MAX_BUSES; ++busno) {
        for (devno = 0; devno < MAX_DEVICES; ++devno) {
            memset(&instance, USBD_CONNECT_WILDCARD, sizeof(usbd_device_instance_t));
            instance.path = busno, instance.devno = devno;
            instance.config = 0;

            /* attach to the device so we can talk to it */
            status = usbd_attach(global_ro_connection, &instance, 0, &usbd_d);

            /* if (status != EOK) { */
            if (status == EOK)
            {
                if (usb_debug >= 2) {
                    usbi_dbg("found a device, bus number: %d, device address: %d", busno, devno);
                }
                
                r = enumerate_device(ctx, &discdevs, busno, devno, usbd_d);
            }
            if (status == EBUSY)
            {
                struct libusb_device *dev;
                dev = get_device_from_session(ctx, busno, devno);
                if (dev)
                {
                    discdevs = discovered_devs_append(*_discdevs, dev);
                    if (!discdevs) 
                        r = LIBUSB_ERROR_NO_MEM;
                    else 
                        *_discdevs = discdevs;
                } else {
                    usbi_warn(ctx, "Found device (bus: %d, dev: %d) but it was busy, "
                              "could not connect to it or obtain from previous session",
                              busno, devno);
                }
            }
        } 
    }

    return 0;
}



/* Get the ACTIVE configuration descriptor for a device.
 *
 * The descriptor should be retrieved from memory, NOT via bus I/O to the
 * device. This means that you may have to cache it in a private structure
 * during get_device_list enumeration. You may also have to keep track
 * of which configuration is active when the user changes it.
 *
 * This function is expected to write len bytes of data into buffer, which
 * is guaranteed to be big enough. If you can only do a partial write,
 * return an error code.
 *
 * This function is expected to return the descriptor in bus-endian format
 * (LE). If it returns the multi-byte values in host-endian format,
 * set the host_endian output parameter to "1".
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if the device is in unconfigured state
 * - another LIBUSB_ERROR code on other failure
 */
static int op_get_device_descriptor(struct libusb_device *dev,
                                    unsigned char *buffer, int *host_endian)
{
    int r = 0;
    struct nto_qnx_device_priv * priv = __device_priv(dev);

    *host_endian = 1;
    memcpy(buffer, priv->dev_descriptor, DEVICE_DESC_LENGTH);
    return r;
}

/* Get the ACTIVE configuration descriptor for a device.
 *
 * The descriptor should be retrieved from memory, NOT via bus I/O to the
 * device. This means that you may have to cache it in a private structure
 * during get_device_list enumeration. You may also have to keep track
 * of which configuration is active when the user changes it.
 *
 * This function is expected to write len bytes of data into buffer, which
 * is guaranteed to be big enough. If you can only do a partial write,
 * return an error code.
 *
 * This function is expected to return the descriptor in bus-endian format
 * (LE). If it returns the multi-byte values in host-endian format,
 * set the host_endian output parameter to "1".
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if the device is in unconfigured state
 * - another LIBUSB_ERROR code on other failure
 */
static int op_get_active_config_descriptor(struct libusb_device *dev,
	unsigned char *buffer, size_t len, int *host_endian)
{
    struct nto_qnx_device_priv *priv = __device_priv(dev);

    if (!priv->config_descriptor)
    {
        return LIBUSB_ERROR_NOT_FOUND;
    }
    memcpy(buffer, priv->config_descriptor, len);
    return 0;
}

/* Get a specific configuration descriptor for a device.
 *
 * The descriptor should be retrieved from memory, NOT via bus I/O to the
 * device. This means that you may have to cache it in a private structure
 * during get_device_list enumeration.
 *
 * The requested descriptor is expressed as a zero-based index (i.e. 0
 * indicates that we are requesting the first descriptor). The index does
 * not (necessarily) equal the bConfigurationValue of the configuration
 * being requested.
 *
 * This function is expected to write len bytes of data into buffer, which
 * is guaranteed to be big enough. If you can only do a partial write,
 * return an error code.
 *
 * This function is expected to return the descriptor in bus-endian format
 * (LE). If it returns the multi-byte values in host-endian format,
 * set the host_endian output parameter to "1".
 *
 * Return 0 on success or a LIBUSB_ERROR code on failure.
 */
static int op_get_config_descriptor(struct libusb_device *dev,
	uint8_t config_index, unsigned char *buffer, size_t len, int *host_endian)
{
    int r = 0;
    /* int devno, busno, ifno; */

    struct nto_qnx_device_priv * dpriv = __device_priv(dev);

    /* write selected config to buffer */
    r = usbd_descriptor(dpriv->usbd_device, 0, USB_DESC_CONFIGURATION,
                        USB_RECIPIENT_DEVICE, config_index, 0, buffer, len);

    /* Not fully reading the descriptor is fine, as it's a usual thing
       to do if there are extra descriptors */
    if(r == EMSGSIZE)
        r = 0;
    
    return qnx_err_to_libusb(r);
}


/* Open a device for I/O and other USB operations. The device handle
 * is preallocated for you, you can retrieve the device in question
 * through handle->dev.
 *
 * Your backend should allocate any internal resources required for I/O
 * and other operations so that those operations can happen (hopefully)
 * without hiccup. This is also a good place to inform libusb that it
 * should monitor certain file descriptors related to this device -
 * see the usbi_add_pollfd() function.
 *
 * This function should not generate any bus I/O and should not block.
 *
 * This function is called when the user attempts to obtain a device
 * handle for a device.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_ACCESS if the user has insufficient permissions
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since
 *   discovery
 * - another LIBUSB_ERROR code on other failure
 *
 * Do not worry about freeing the handle on failed open, the upper layers
 * do this for you.
 */

static int op_open(struct libusb_device_handle *handle)
{
    /* === libusb === */
    struct libusb_device * dev = handle->dev;
    struct nto_qnx_device_handle_priv * hpriv = __device_handle_priv(handle);
    struct nto_qnx_device_priv * dpriv = __device_priv(handle->dev);

    /* === internal === */
    int r = 0;
    int status;
    int devno, busno, ifno;
    struct usbd_connection * usbd_c;
    struct usbd_device * usbd_d;
    usbd_device_instance_t instance;

    /* vars required for getting the control pipe */
    usbd_descriptors_t * ud;
    struct usbd_desc_node * usbd_dn;
    struct usbd_pipe * u_pipe;

    /* find internal device struct, don't care which interface */
    busno = dev->bus_number, devno = dev->device_address, ifno = 0;

    usbd_c = dpriv->connection;
    usbd_d = dpriv->usbd_device;
    
    /* Check if we have write privileges  */
    usbi_dbg ("Checking if we have write privileges");

    if (usbd_c == global_ro_connection) /* compare pointers */
    {
        /* The device may have been connected through the use of a
           connection to the usb stack which is read only, so try
           detaching and re-attaching using the read-write connection
           instead */

        /* === detaching === */
        /* TODO: ensure that no io is occuring otherwise this will fail */
        if (usbd_d != NULL)
        {
            usbi_dbg ("Detaching usbd_device, since it is probably read only");

            status = usbd_detach(usbd_d);
            if (status != EOK) {
                usbi_err(HANDLE_CTX(handle), "usbd_detach() failed, error = %s", strerror(status));
                return qnx_err_to_libusb(status);
            }
        }

        /* === Re-attaching === */

        /* Make an instance manually */
        usbi_dbg ("Now working on attaching/reattaching the device to the stack");
        memset(&instance, USBD_CONNECT_WILDCARD, sizeof(usbd_device_instance_t));
        instance.path = busno, instance.devno = devno;

        status = usbd_attach(global_connection, &instance, 0, &usbd_d);
        if (status != EOK) {
            dpriv->usbd_device = NULL;
            if (usb_debug >= 2) {
                usbi_info(HANDLE_CTX(handle), "usbd_attach() failed, error = %s", strerror(status));
            }
            return qnx_err_to_libusb(status);
        }
        else 
        {
            /* Successfully connected, thereby upgrading privileges to
               read-write. Now update structures. */
            dpriv->usbd_device = usbd_d;
            dpriv->connection = global_connection;
        }

    }

    /* get a pointer to the control endpoint descriptor */
    ud = (usbd_descriptors_t *) usbd_endpoint_descriptor(usbd_d, 1, 0, 0, 0,
                                                         &usbd_dn);
    if (ud == 0) {

        usbi_err(HANDLE_CTX(handle), "usbd_endpoint_descriptor() failed");
        /* TODO: cleanup */
        return LIBUSB_ERROR_ACCESS;
    }

    /* open the control pipe */
    status = usbd_open_pipe(usbd_d, ud, &u_pipe);
    if (status != EOK) {
        usbi_err(HANDLE_CTX(handle), "usbd_open_pipe() failed");
        /* TODO: cleanup */
        return qnx_err_to_libusb(status);
    }
    hpriv->control_pipe = u_pipe;

    /* Creating a pipe representing this device */
    pipe(hpriv->fds);

    /* We DO NOT want to block when writing to this pipe */
    fcntl(hpriv->fds[1], F_SETFL, O_NONBLOCK);
    /* we DO NOT want to block when reading from this pipe */
    fcntl(hpriv->fds[0], F_SETFL, O_NONBLOCK);

    usbi_dbg("Calling usbi_add_pollfd");
    usbi_add_pollfd(HANDLE_CTX(handle), hpriv->fds[0], POLLIN);

    usbi_info(HANDLE_CTX(handle), "device open for access");

    return r;
}

/* Close a device such that the handle cannot be used again. Your backend
 * should destroy any resources that were allocated in the open path.
 * This may also be a good place to call usbi_remove_pollfd() to inform
 * libusb of any file descriptors associated with this device that should
 * no longer be monitored.
 *
 * This function is called when the user closes a device handle.
 */
static void op_close(struct libusb_device_handle *handle)
{
    usbi_dbg("starting op_close");
    struct libusb_device *dev = handle->dev;
    struct nto_qnx_device_handle_priv * hpriv = __device_handle_priv(handle);
    struct nto_qnx_device_priv * dpriv = __device_priv(dev);
    usbd_device_instance_t instance;

    int devno, busno, ifno;
    busno = dev->bus_number, devno = dev->device_address, ifno = 0;

    int status;

    if(hpriv == NULL)
      return;

    /* release interfaces */
    usbi_dbg("releasing all claimed interfaces");
    while(!(TAILQ_EMPTY(&dpriv->claimed_interfaces)))
    {
        struct claimed_interfaces_list * node = 
            TAILQ_FIRST(&dpriv->claimed_interfaces);

        TAILQ_REMOVE(&dpriv->claimed_interfaces, node, chain);
        free(node);
    }

    /* cleanup pipe */
    usbi_dbg("closing pipe");
    if (hpriv->control_pipe != NULL) {
        usbd_reset_pipe(hpriv->control_pipe);
        status = usbd_close_pipe(hpriv->control_pipe);
        if (status != EOK)
        {
            usbi_err(HANDLE_CTX(handle), "could not close pipe, error = %s", strerror(status));
        } else {
            hpriv->control_pipe = NULL;
        }
    }

    /* detatch from r/w connection and connect to ro */
    if (dpriv->connection == global_connection) /* compare pointers */
    {
        if (dpriv->usbd_device != NULL)
        {
            usbi_dbg("Detaching usb device from r/w connection");
            status = usbd_detach(dpriv->usbd_device);
            if (status != EOK)
            {
                usbi_err(HANDLE_CTX(handle), "usbd_detach() failed, error = %s", strerror(status));
                return;
            }
        }

        usbi_dbg("Now working on reattaching to r/o connection");
        memset(&instance, USBD_CONNECT_WILDCARD, sizeof(usbd_device_instance_t));
        instance.path = busno, instance.devno = devno;
            
        status = usbd_attach(global_ro_connection, &instance, 0, &dpriv->usbd_device);
        if (status != EOK)
        {
            dpriv->usbd_device = NULL;
            usbi_dbg("usbd_attach() failed, error = %s", strerror(status));
            return;
        }
        else
        {
            dpriv->connection = global_ro_connection;
        }
    }

    /* remove fds */
    if (hpriv->fds[0] != -1)
    {
        usbi_dbg("removing file descriptors from list");
        usbi_remove_pollfd(HANDLE_CTX(handle), hpriv->fds[0]);
        close (hpriv->fds[1]);
        close (hpriv->fds[0]);

        hpriv->fds[0] = hpriv->fds[1] = -1;
    }

}

/* ============ Submissions ============ */

/* Submit a control transfer based on the `itransfer' struct */
static int submit_control_transfer(struct usbi_transfer *itransfer)
{

    /* === libusb === */
    dump_itransfer_info(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
    struct libusb_transfer *transfer = 
        USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_device_handle_priv *hpriv =
        __device_handle_priv(transfer->dev_handle);
    struct libusb_control_setup * setup = libusb_control_transfer_get_setup(transfer);

    
    /* === internal === */
    struct usbd_pipe * control_pipe;
    struct usbd_urb * urb;

    int status;

    uint8_t requesttype = setup->bmRequestType;
    uint8_t request = setup->bRequest;

    /* May need to fix the endianness of these */
    uint16_t value = setup->wValue;
    uint16_t w_index = setup->wIndex;
    uint16_t len = setup->wLength;
    int dir = (requesttype & USB_ENDPOINT_IN) ? URB_DIR_IN : URB_DIR_OUT;

    /* This holds the libusb buffer which will either capture or
       provide the data (depending on whether the message direction is
       in or out).

       REMEMBER: the control transfer buffer starts with the setup
       packet!!! The data portion comes later*/
    unsigned char * bytes = libusb_control_transfer_get_data(transfer);

    /* The pointer to usbd's data buffer, should be allocated with
       usbd_alloc and freed with usbd_free */
    unsigned char * usbd_buffer;

    if (dir == URB_DIR_IN)
    {
        usbi_dbg ("submit_control_transfer: direction in");
    } else 
    {
        usbi_dbg ("submit_control_transfer: direction out");
    }

    urb = usbd_alloc_urb(0);
    if (urb == 0) {
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_alloc_urb() failed");
        /* TODO: should we fail noisily or just exit? */
    }
    tpriv->urb = urb;
    
    /* Double check that its len and not len - LIBUSB_CONTROL_SETUP_SIZE */
    usbi_info(ITRANSFER_CTX(itransfer), "allocating memory for control transfer, len = %d", len);
    usbd_buffer = usbd_alloc(len);
    if (usbd_buffer == 0) {
        usbi_dbg("usbd_alloc() failed");
        usbd_free_urb(urb);
        return LIBUSB_ERROR_NO_MEM;
        /* TODO: should we fail noisily or just exit? */
    }

    /* Now that it's valid, save location of usbd_buffer for
       reaping */
    tpriv->internal_buffer = usbd_buffer;

    /* If we are outputting to the device, copy over the
       information */
    if (dir == URB_DIR_OUT) {
        memcpy(usbd_buffer, bytes, len);
    }
    status = usbd_setup_vendor(urb,
                               dir | URB_SHORT_XFER_OK,
                               request,
                               requesttype & 0x7F,
                               value,
                               w_index,
                               usbd_buffer,
                               len);
    if (status != EOK) {
        usbd_free_urb(urb);
        usbd_free(usbd_buffer);
        /* TODO: error reporting */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_setup_vendor failed");
        return qnx_err_to_libusb(status);
    }

    /* printf ("something asdf asdf asd\n"); */

    /* TODO: cleanup of the transfer should be handled by a callback: qnx_async_io_callback */
    tpriv->transfer_pipe = control_pipe = hpriv->control_pipe;

    status = usbd_io(urb, control_pipe, NULL, 0, CONTROL_TRANSFER_TIMEOUT_MS);
    if (status != EOK) {
        usbd_free_urb(urb);
        usbd_free(usbd_buffer);
        /* pthread_mutex_unlock(&nu_mutex); */
        /* TODO: error reporting */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_io failed");
        return qnx_err_to_libusb(status);
    }

    usbi_info(TRANSFER_CTX(transfer), "control transfer submitted!");

    /* Sync */
    /* control transfers are synchronous in qnx, so we don't need to
       wait to write to the fds */

    uint32_t message = MESSAGE_ASYNC_IO_COMPLETE;
    write (hpriv->fds[1], &message, sizeof (message));
    write (hpriv->fds[1], &itransfer, sizeof (itransfer));

    /* Async */
    /* calloc to ensure contained pointers are NULL */
    /* struct urb_tailq * new_node = (struct urb_tailq *)calloc(1, sizeof(struct urb_tailq)); */
    /* new_node->itransfer = itransfer; */
    
    /* TAILQ_INSERT_TAIL(&urb_head, new_node, chain); */

    /* /\* Send pulse to wakeup polling thread *\/ */
    /* MsgSendPulse(urb_main_coid, */
    /*              -1,            /\* same priority as calling thread*\/ */
    /*              0,             /\* doesn't matter what pulse code *\/ */
    /*              0);            /\* doesn't matter what payload *\/ */

    return 0;
}



static int submit_bulk_transfer(struct usbi_transfer *itransfer, unsigned char urb_type)
{
    /* see submit_control_transfer for comments on what this stuff does */
    /* === libusb === */
    dump_itransfer_info(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct libusb_device_handle *dev_handle = transfer->dev_handle;
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(dev_handle);
    struct nto_qnx_device_priv *dpriv = __device_priv(dev_handle->dev);
    
    /* === internal === */
    struct usbd_desc_node * usbd_dn;
    struct usbd_pipe * bulk_pipe;
    struct usbd_urb * urb;

    int status;

    unsigned char * bytes = transfer->buffer;
    int size = transfer->length;
    int timeout = transfer->timeout ? transfer->timeout : USBD_TIME_INFINITY;
    int dir = (transfer->endpoint & LIBUSB_ENDPOINT_IN) ? URB_DIR_IN : URB_DIR_OUT;
    int intr = urb_type == NTO_QNX_URB_TYPE_INTERRUPT;
    
    usbd_descriptors_t * ud;

    unsigned char * usbd_buffer;

    /* Check the endpoint address */
    if (dir == URB_DIR_IN) {
        if ((transfer->endpoint & USB_ENDPOINT_IN) != USB_ENDPOINT_IN) {
            /* TODO: handle this kind of error */
            usbi_err(ITRANSFER_CTX(itransfer), "read on endpoint address ep=%x", transfer->endpoint);
            return LIBUSB_ERROR_INVALID_PARAM;
        }
    } else {
        if ((transfer->endpoint & USB_ENDPOINT_IN) == USB_ENDPOINT_IN) {
            /* TODO: handle this kind of error */
            usbi_err(ITRANSFER_CTX(itransfer), "write on endpoint address ep=%x", transfer->endpoint);
            return LIBUSB_ERROR_INVALID_PARAM;
        }
    }

    int config = dpriv->selected_configuration;
    if ( config == -1 ) 
    {
        usbi_dbg("Didn't know what configuration was being used,"
                 " sending out control transfer to request the information");
        /* if we haven't explicitly selected a configuration, find out
           what configuration the device is using */
        libusb_get_configuration(dev_handle, &config);
        dpriv->selected_configuration = config;
    }

    int ifno = hpriv->interface_number;
    if ( ifno == -1 )
    {
        usbi_dbg("Didn't know the interface number,"
                 " but it should have been claimed. Trying to DWYM with interface 0.");
        /* well an interface SHOULD have been selected but just
           incase we will assume interface 0 */
        ifno = 0;
    }

    int alt = hpriv->alt;
    if ( alt == -1 )
    {
        usbi_dbg("Didn't know what alt setting to use,"
                 " but it should have been set by select_interface. Trying to DWYM with alt 0.");
        /* if no explicit altsetting, assume 0 */
        alt = 0;
    }

    ud = (usbd_descriptors_t *) usbd_endpoint_descriptor( dpriv->usbd_device,
                                                          config,
                                                          ifno,
                                                          alt,
                                                          transfer->endpoint,
                                                          &usbd_dn );

    if (ud == 0) {
        /* TODO: support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_endpoint_descriptor failed");
        usbi_dbg("usbd_endpoint_descriptor args: "
                 "dpriv->usbd_device: %x, "
                 "config: %x, "
                 "ifno: %d, "
                 "alt: %d, "
                 "transfer->endpoint: %x",
                 dpriv->usbd_device, config, ifno, alt, transfer->endpoint);
        /* TODO: what kind of error should this be? */
        return LIBUSB_ERROR_OTHER;
    }

    status = usbd_open_pipe(dpriv->usbd_device, ud, &bulk_pipe);
    if (status != EOK) {
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_open_pipe failed");
        /* TODO: what kind of error should this be? */
        return LIBUSB_ERROR_OTHER;
    }
    /* Store transfer pipe so we can close it at IO reaping */
    tpriv->transfer_pipe = bulk_pipe;

    urb = usbd_alloc_urb(0);
    if (urb == 0) {
        usbd_close_pipe(bulk_pipe);
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_alloc_urb failed, freeing pipe");
        return LIBUSB_ERROR_NO_MEM;
    }
    /* Keep track of the urb so that we can free it later */
    tpriv->urb = urb;

    usbd_buffer = usbd_alloc(size);
    if (usbd_buffer == 0) {
        usbd_free_urb(urb);
        usbd_close_pipe(bulk_pipe);
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_alloc failed, freeing pipe and urb");
        return LIBUSB_ERROR_NO_MEM;
    }

    /* save location of usbd_buffer for reaping */
    tpriv->internal_buffer = usbd_buffer;
    usbi_dbg("allocated internal buffer: %p", usbd_buffer);

    if (dir == URB_DIR_OUT) {
        memcpy(usbd_buffer, bytes, size);
    }
    if (intr) {
        status = usbd_setup_interrupt(urb, dir | URB_SHORT_XFER_OK, usbd_buffer, size);
    } else {
        status = usbd_setup_bulk(urb, dir | URB_SHORT_XFER_OK, usbd_buffer, size);
    }
    if (status != EOK) {
        usbd_free_urb(urb);
        usbd_free(usbd_buffer);
        usbd_close_pipe(bulk_pipe);
        /* TODO: support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_setup_bulk failed, freeing urb, usbd_buffer and pipe");
        return qnx_err_to_libusb(status);
    }

    status = usbd_io( urb,
                      bulk_pipe,
                      qnx_async_io_callback,
                      (void *) itransfer,
                      timeout );

    if (status != EOK) {
        usbd_free_urb(urb);
        usbd_free(usbd_buffer);
        usbd_close_pipe(bulk_pipe);
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "bulk usbd_io failed, freeing urb, usbd_buffer, and pipe");
        return qnx_err_to_libusb(status);
    }

    usbi_info(TRANSFER_CTX(transfer), "submit_bulk_transfer: bulk transfer submitted!");

    /* struct urb_tailq * new_node = (struct urb_tailq *)calloc(1, sizeof(struct urb_tailq)); */
    /* new_node->itransfer = itransfer; */

    /* TAILQ_INSERT_TAIL(&urb_head, new_node, chain); */

    /* /\* Send pulse to wakeup polling thread if it's sleeping *\/ */
    /* MsgSendPulse(urb_main_coid,  */
    /*              -1,            /\* same priority as calling thread*\/  */
    /*              0,             /\* doesn't matter what pulse code *\/ */
    /*              0);            /\* doesn't matter what payload *\/ */

    return 0;
}

static int submit_iso_transfer(struct usbi_transfer *itransfer)
{

    /* === libusb === */
    usbi_dbg("itransfer pointer = %p", itransfer);
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
    struct libusb_device_handle *dev_handle = transfer->dev_handle;
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(dev_handle);
    struct nto_qnx_device_priv *dpriv = __device_priv(dev_handle->dev);

    /* === internal === */
    struct usbd_urb * urb;
    struct usbd_desc_node * usbd_dn;
    struct usbd_pipe * iso_pipe;
    usbd_descriptors_t * ud;
    int dir = (transfer->endpoint & LIBUSB_ENDPOINT_IN) ? URB_DIR_IN : URB_DIR_OUT;

    char * bytes = transfer->buffer;
    unsigned char * usbd_buffer;
    

    int status;

    /* 1. check the endpoint address */
    if (dir == URB_DIR_IN) {
        if ((transfer->endpoint & USB_ENDPOINT_IN) != USB_ENDPOINT_IN) {
            /* TODO: handle this kind of error */
            usbi_err(ITRANSFER_CTX(itransfer), "read on endpoint address ep=%x", transfer->endpoint);
            return LIBUSB_ERROR_INVALID_PARAM;
        }
    } else {
        if ((transfer->endpoint & USB_ENDPOINT_IN) == USB_ENDPOINT_IN) {
            /* TODO: handle this kind of error */
            usbi_err(ITRANSFER_CTX(itransfer), "write on endpoint address ep=%x", transfer->endpoint);
            return LIBUSB_ERROR_INVALID_PARAM;
        }
    }

    /* 2.1 confirm configuration */

    int config = dpriv->selected_configuration;
    if ( config == -1 ) 
    {
        usbi_dbg("Didn't know what configuration to use,"
                 " sending out control transfer to figure it out.");
        /* if we haven't explicitly selected a configuration, find out
           what configuration the device is using */
        libusb_get_configuration(dev_handle, &config);
        dpriv->selected_configuration = config;
    }

    int ifno = hpriv->interface_number;
    if ( ifno == -1 )
    {
        usbi_dbg("Didn't know what interface setting to use,"
                 " but it should have been set. Trying to DWYM with interface 0.");

        /* well an interface SHOULD have been selected but just
           incase we will assume interface 0 */
        ifno = 0;
    }

    int alt = hpriv->alt;
    if ( alt == -1 )
    {
        usbi_dbg("Didn't know what alt setting to use,"
                 " but it should have been set by select_interface. Trying to DWYM with alt 0.");
        /* if no explicit altsetting, assume 0 */
        alt = 0;
    }

    /* 2.2 Determine endpoint */
    ud = (usbd_descriptors_t *) usbd_endpoint_descriptor( dpriv->usbd_device,                          
                                                          config,
                                                          ifno,
                                                          alt,
                                                          transfer->endpoint,
                                                          &usbd_dn );
    
    if (ud == 0) {
        /* TODO: support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_endpoint_descriptor failed");
        /* TODO: what kind of error should this be? */
        return LIBUSB_ERROR_OTHER;
    }

    /* 3. open pipe */
    status = usbd_open_pipe(dpriv->usbd_device, ud, &iso_pipe);
    if (status != EOK) {
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_open_pipe failed");
        /* TODO: what kind of error should this be? */
        return LIBUSB_ERROR_OTHER;
    }
    
    tpriv->transfer_pipe = iso_pipe;

    /* 4. allocate urb */
    urb = usbd_alloc_urb(0);
    if (urb == 0) {
        usbd_close_pipe(iso_pipe);
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_alloc_urb failed, freeing pipe");
        return LIBUSB_ERROR_NO_MEM;
    }
    tpriv->urb = urb;

    /* 5. allocate usbd_buffer */
    usbd_buffer = usbd_alloc(transfer->length);
    if (usbd_buffer == 0) {
        usbd_free_urb(urb);
        usbd_close_pipe(iso_pipe);
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_alloc failed, freeing pipe and urb");
        return LIBUSB_ERROR_NO_MEM;
    }

    /* save location of usbd_buffer for reaping */
    tpriv->internal_buffer = usbd_buffer;
    usbi_dbg("allocated internal buffer: %p", usbd_buffer);

    if (dir == URB_DIR_OUT) {
        memcpy(usbd_buffer, bytes, transfer->length);
    }

    status = usbd_setup_isochronous( urb,
                                     dir | URB_SHORT_XFER_OK | URB_ISOCH_ASAP,
                                     0, /* ignored due to URB_ISOCH_ASAP */
                                     usbd_buffer,
                                     transfer->length );

    if (status != EOK) {
        usbd_free_urb(urb);
        usbd_free(usbd_buffer);
        usbd_close_pipe(iso_pipe);
        /* TODO: support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "usbd_setup_isochronous failed,"
                 " freeing urb, usbd_buffer and pipe");
        return qnx_err_to_libusb(status);
    }


    status = usbd_io( urb,
                      iso_pipe,
                      qnx_async_io_callback,
                      (void *) itransfer,
                      transfer->timeout );

    if (status != EOK) {
        usbd_free_urb(urb);
        usbd_free(usbd_buffer);
        usbd_close_pipe(iso_pipe);
        /* TODO: Support this kind of error */
        usbi_err(ITRANSFER_CTX(itransfer), "isochronous usbd_io failed,"
                 " freeing urb, usbd_buffer, and pipe");
        return qnx_err_to_libusb(status);
    }

    return 0;
}
	
/* Submit a transfer. Your implementation should take the transfer,
 * morph it into whatever form your platform requires, and submit it
 * asynchronously.
 *
 * This function must not block.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
 * - another LIBUSB_ERROR code on other failure
 */
static int op_submit_transfer(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer =
		USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

	switch (transfer->type) {
	case LIBUSB_TRANSFER_TYPE_CONTROL:
		return submit_control_transfer(itransfer);
	case LIBUSB_TRANSFER_TYPE_BULK:
		return submit_bulk_transfer(itransfer, NTO_QNX_URB_TYPE_BULK);
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		return submit_bulk_transfer(itransfer, NTO_QNX_URB_TYPE_INTERRUPT);
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		return submit_iso_transfer(itransfer);
	default:
		usbi_err(TRANSFER_CTX(transfer),
			"unknown endpoint type %d", transfer->type);
		return LIBUSB_ERROR_INVALID_PARAM;
	}
}

static void qnx_handle_callback(struct usbi_transfer *itransfer)
{
    usbi_info(ITRANSFER_CTX(itransfer), "handling callbacks");
    usbi_info(ITRANSFER_CTX(itransfer), "handling itransfer = %p", itransfer);
    dump_itransfer_info(itransfer);
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(transfer->dev_handle);
    struct nto_qnx_device_priv *dpriv = __device_priv(transfer->dev_handle->dev);

    /* Determine io direction */
    unsigned char * bytes;
    _uint32 ustatus;
    _uint32 usize;

    int status;

    int dir;
    if(transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL) {
        struct libusb_control_setup * setup = libusb_control_transfer_get_setup(transfer);
        uint8_t requesttype = setup->bmRequestType;
        dir = ((requesttype & USB_ENDPOINT_IN) == USB_ENDPOINT_IN) ? URB_DIR_IN : URB_DIR_OUT;
    } else {
        /* Non control transfer so use the endpoint to work out direction. */
        uint8_t endpoint = transfer->endpoint;
        dir = ((endpoint & LIBUSB_ENDPOINT_IN) == LIBUSB_ENDPOINT_IN) ? URB_DIR_IN : URB_DIR_OUT;
    }

    status = usbd_urb_status(tpriv->urb, &ustatus, &usize);

    /* Handle the fact that control transfers have the setup packet as
       part of the buffer */
    if (transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL)
    {
        bytes = libusb_control_transfer_get_data(transfer);
    } else {
        bytes = transfer->buffer;
    }

    usbi_info(ITRANSFER_CTX(itransfer), "Checking if we need to clear the pipe after a stall");

    /* If the transfer caused a stall we must clear the pipe */
    /* The windows strategy */
    if ((ustatus & USBD_USB_STATUS_MASK) == USBD_STATUS_STALL)
    {
        usbi_info(ITRANSFER_CTX(itransfer), "Clearing pipe...");
        status = usbd_reset_pipe(tpriv->transfer_pipe);
        if (status != EOK) 
        { 
            usbi_info(ITRANSFER_CTX(itransfer),
                      "Error (%s), couldn't reset pipe,"
                      " resetting device anyways.", strerror(status));
        }
        status = usbd_reset_device(dpriv->usbd_device);
        if (status != EOK) 
        {
            usbi_info(ITRANSFER_CTX(itransfer),
                      "Error (%s), couldn't "
                      "reset device.", strerror(status));
        }
    }

    /* If the transfer went well, we need to indicate that to libusb */
    if (status == EOK)
    {
        usbi_info(ITRANSFER_CTX(itransfer),
                  "status = SUCCESS... ustatus = %x, usize = %d",
                  ustatus, usize);

        itransfer->transferred += usize;

        usbi_info(ITRANSFER_CTX(itransfer),
                  "Checking the direction of the transfer, then "
                  "copying the bytes if needed");

        /* Might as well copy whatever we can... 
           At this point we still haven't checked if the transfer has
           completed without error*/
        if (dir == URB_DIR_IN)
        {
            memcpy(bytes, tpriv->internal_buffer, usize);
        }
    } 
    else
    {
        usbi_info(ITRANSFER_CTX(itransfer), "status FAILURE... ustatus = %x, usize = %d errstatus: %s",
                  ustatus, usize, strerror(status));
        /* TODO: Error, usbd_urb_status failed */
    }

    /* Cleanup */
    usbi_dbg("tpriv->urb: %p", tpriv->urb);
    usbd_free_urb(tpriv->urb);
    usbi_dbg("tpriv->internal_buffer: %p", tpriv->internal_buffer);
    usbd_free(tpriv->internal_buffer);
    usbi_dbg("Internal buffer freed");
    
    /* If the transfer pipe was not the control pipe */
    usbi_info(ITRANSFER_CTX(itransfer), "Do we need to close the pipe?");
    if (!(tpriv->transfer_pipe == hpriv->control_pipe))
    {
        usbi_info(ITRANSFER_CTX(itransfer), "Yes we need to close the pipe");
        usbd_close_pipe(tpriv->transfer_pipe);
    }

    /* TODO: handle canceled transfers */

    usbi_info(ITRANSFER_CTX(itransfer), "Transfer completed");
    usbi_handle_transfer_completion(itransfer, qnx_transfer_status(itransfer, ustatus));
    /* usbi_handle_transfer_completion(itransfer, LIBUSB_TRANSFER_COMPLETED); */
}

/* This should only be used as a callback for bulk submissions at this
   point. It may be useful later for other functions. */
static void qnx_async_io_callback(struct usbd_urb * urb, struct usbd_pipe * upipe, void * userdata)
{
    usbi_dbg ("handling async callback");
    
    struct usbi_transfer *itransfer = (struct usbi_transfer *) userdata;
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(transfer->dev_handle);

    uint32_t message;

    _uint32 ustatus;
    _uint32 usize;
    int ustatus_masked;
    _uint32 status;

    status = usbd_urb_status(urb, &ustatus, &usize);
    ustatus_masked = (ustatus & USBD_URB_STATUS_MASK);
    if ((ustatus & USBD_URB_STATUS_MASK) == USBD_STATUS_CMP)
    {
        usbi_dbg ("USBD_STATUS_CMP FOUND!");
    }

    if ((ustatus & USBD_USB_STATUS_MASK) == USBD_STATUS_STALL)
    {
        usbd_reset_pipe(tpriv->transfer_pipe);
    }

    usbi_info(ITRANSFER_CTX(itransfer), "async transfer status report, "
              "USBD_USB_STATUS_MASK = %x ustatus = %x ustatus_masked = %x, stderr = %s",
              USBD_USB_STATUS_MASK,
              ustatus,
              ustatus_masked,
              strerror(status));

    /* send a completion message to the device's file descriptor */
    message = MESSAGE_ASYNC_IO_COMPLETE;
    write (hpriv->fds[1], &message, sizeof (message));
    write (hpriv->fds[1], &itransfer, sizeof (itransfer));
}

/* Handle any pending events. This involves monitoring any active
 * transfers and processing their completion or cancellation.
 *
 * The function is passed an array of pollfd structures (size nfds)
 * as a result of the poll() system call. The num_ready parameter
 * indicates the number of file descriptors that have reported events
 * (i.e. the poll() return value). This should be enough information
 * for you to determine which actions need to be taken on the currently
 * active transfers.
 *
 * For any cancelled transfers, call usbi_handle_transfer_cancellation().
 * For completed transfers, call usbi_handle_transfer_completion().
 * For control/bulk/interrupt transfers, populate the "transferred"
 * element of the appropriate usbi_transfer structure before calling the
 * above functions. For isochronous transfers, populate the status and
 * transferred fields of the iso packet descriptors of the transfer.
 *
 * This function should also be able to detect disconnection of the
 * device, reporting that situation with usbi_handle_disconnect().
 *
 * When processing an event related to a transfer, you probably want to
 * take usbi_transfer.lock to prevent races. See the documentation for
 * the usbi_transfer structure.
 *
 * Return 0 on success, or a LIBUSB_ERROR code on failure.
 */
static int op_handle_events(struct libusb_context *ctx, struct pollfd *fds, nfds_t nfds, int num_ready) 
{
    usbi_dbg("op_handle_events: Starting os specific handle events");
    usbi_info(ctx, "num_ready = %d", num_ready);
    struct usbi_transfer *itransfer;

    int i, ret;
    uint32_t message;

    pthread_mutex_lock(&ctx->open_devs_lock);
    
    for (i = 0; i < nfds && num_ready > 0; ++i)
    {
        struct pollfd *pollfd = &fds[i];
        struct libusb_device_handle *handle;
        struct nto_qnx_device_handle_priv *hpriv = NULL;

        usbi_info(ctx, "checking fds %i with revents = %x", fds[i], pollfd->revents);

        if (!pollfd->revents)
        continue;
        
        --num_ready;
        list_for_each_entry(handle, &ctx->open_devs, list, libusb_device_handle) 
        {
            hpriv = __device_handle_priv(handle);
            if (hpriv->fds[0] == pollfd->fd)
            break;
        }

        if (NULL == hpriv)
        {
            usbi_err (ctx, "fd %d is not an event pipe!",pollfd->fd);
            pthread_mutex_unlock(&ctx->open_devs_lock);
            return ENOENT;
        }
        /* usbi_info(ctx, "complete_transfers is %d", hpriv->complete_transfers); */
        /* we should reap io at least hpriv->complete_transfers times */
        /* int complete_transfers = hpriv->complete_transfers; */

        /* TODO: Should simply read until there remains nothing in the pipe */

        /* int j; */
        /* for (j = 0; j < complete_transfers; ++j) */
        /* { */

        int more_msgs = 1;   /* bool: are there messages to read in the pipe */
        while (more_msgs)
        {
            /* --hpriv->complete_transfers; */
        
            /* If there are no errors, read from the pipe */
            /* Once read, if there was nothing, continue */
            /* Once read, if there was an error, check errno for EAGAIN or EWOULDBLOCK */
            /*    if EAGAIN or EWOULDBLOCK then we are done here, return */
            /*    otherwise, continue reading */

            if (!(pollfd->revents & POLLERR)) {
                usbi_info (ctx, "About to read...");
                ret = read (hpriv->fds[0], &message, sizeof (message));
                usbi_info (ctx, "Finished reading...");
                if ( ret < 0 ){
                    more_msgs = 0;
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                    {
                        usbi_info (ctx, "Done reading!!");
                    }
                    break;
                }
                if (ret < sizeof (message)) continue;
            } else message = MESSAGE_DEVICE_GONE;
        
            switch (message) 
            {
            case MESSAGE_DEVICE_GONE:
                /* TODO: handle device gone */
                usbi_dbg("op_handle_events: MESSAGE_DEVICE_GONE stub code");
                usbi_remove_pollfd(HANDLE_CTX(handle), hpriv->fds[0]);
                usbi_handle_disconnect(handle);
                continue;
            case MESSAGE_ASYNC_IO_COMPLETE:
                read (hpriv->fds[0], &itransfer, sizeof(itransfer));
                qnx_handle_callback (itransfer);
                break;
            default:
                usbi_err (ctx, "unkown message received from device pipe");
            }
            usbi_info(ctx, "going for another read...");
            /* usbi_info(ctx, "complete_transfers reduced to %d", hpriv->complete_transfers); */
        }
        
    }

    pthread_mutex_unlock(&ctx->open_devs_lock);
    
    return 0;
}


/* Claim an interface. When claimed, the application can then perform
 * I/O to an interface's endpoints.
 *
 * This function should not generate any bus I/O and should not block.
 * Interface claiming is a logical operation that simply ensures that
 * no other drivers/applications are using the interface, and after
 * claiming, no other drivers/applicatiosn can use the interface because
 * we now "own" it.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if the interface does not exist
 * - LIBUSB_ERROR_BUSY if the interface is in use by another driver/app
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_claim_interface(struct libusb_device_handle *handle, int iface)
{

    int r;
    struct libusb_config_descriptor *config;
    struct nto_qnx_device_priv *dpriv = __device_priv(handle->dev);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(handle);
    struct claimed_interfaces_list * node;

    /* Does this interface exist? */
    r = libusb_get_active_config_descriptor(handle->dev, &config);
    if (r != 0)
    {
        return r;
    }
    usbi_dbg("config->bNumInterfaces = %d, iface = %d", config->bNumInterfaces, iface);
    int iface_found = iface < config->bNumInterfaces;
    free(config);
    if (!iface_found)
    {
        return LIBUSB_ERROR_NOT_FOUND;
    }

    /* Find out if we claimed it already */
    int used = 0;
    TAILQ_FOREACH(node, &dpriv->claimed_interfaces, chain)
    {
        if (node->claimed_interface == iface)
        {
            /* already claimed! */
            used = 1;
        }
    }
    
    /* If we haven't claimed it already, claim it */
    if (!used)
    {
        /* Calloc to ensure that the TAILQ pointers are null */
        struct claimed_interfaces_list * new_node = 
            (struct claimed_interfaces_list *)calloc(1, sizeof(struct claimed_interfaces_list));

        TAILQ_INSERT_TAIL(&dpriv->claimed_interfaces, new_node, chain);

        hpriv->interface_number = iface;
        return 0;
    } 
    else
    {
        return LIBUSB_ERROR_BUSY;
    }

    /* TODO: should this try and claim the interface through the
       underlying driver? Probably*/

    return 0;
}

/* Release a previously claimed interface.
 *
 * This function should also generate a SET_INTERFACE control request,
 * resetting the alternate setting of that interface to 0. It's OK for
 * this function to block as a result.
 *
 * You will only ever be asked to release an interface which was
 * successfully claimed earlier.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_release_interface(struct libusb_device_handle *handle, int iface)
{
    int r;
    struct nto_qnx_device_priv *dpriv = __device_priv(handle->dev);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(handle);
    struct claimed_interfaces_list * r_node = NULL;
    struct claimed_interfaces_list * node;

    TAILQ_FOREACH(node, &dpriv->claimed_interfaces, chain)
    {
        if (node->claimed_interface == iface)
        {
            r_node = node;
        }
    }

    if (r_node != NULL)
    {
        TAILQ_REMOVE(&dpriv->claimed_interfaces, r_node, chain);
    }

    /* TODO: select the default interface with usbd_select_interface */
    hpriv->interface_number = -1;
    hpriv->alt = -1;
    r = usbd_select_interface(dpriv->usbd_device, 0, 0);
    
    return qnx_err_to_libusb(r);
}

/* Get the bConfigurationValue for the active configuration for a device.
 * Optional. This should only be implemented if you can retrieve it from
 * cache (don't generate I/O).
 *
 * If you cannot retrieve this from cache, either do not implement this
 * function, or return LIBUSB_ERROR_NOT_SUPPORTED. This will cause
 * libusb to retrieve the information through a standard control transfer.
 *
 * This function must be non-blocking.
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - LIBUSB_ERROR_NOT_SUPPORTED if the value cannot be retrieved without
 *   blocking
 * - another LIBUSB_ERROR code on other failure.
 */
static int op_get_configuration(struct libusb_device_handle *handle, int *config)
{
    struct nto_qnx_device_priv *dpriv = __device_priv(handle->dev);

    /* Not available in cache necessarily  */
    if (dpriv->selected_configuration == -1)
    {
        return LIBUSB_ERROR_NOT_SUPPORTED;
    }
    else
    {
        *config = dpriv->selected_configuration;
        return 0;
    }
}

/* Set the active configuration for a device.
 *
 * A configuration value of -1 should put the device in unconfigured state.
 *
 * This function can block.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if the configuration does not exist
 * - LIBUSB_ERROR_BUSY if interfaces are currently claimed (and hence
 *   configuration cannot be changed)
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure.
 */
static int op_set_configuration(struct libusb_device_handle *handle, int config)
{

    struct nto_qnx_device_priv *dpriv = __device_priv(handle->dev);
    int r;

    if (!TAILQ_EMPTY(&dpriv->claimed_interfaces))
    {
        /* Cannot select new configuration, interfaces have already
           been claimed */
        return LIBUSB_ERROR_BUSY;
    }

    r = usbd_select_config(dpriv->usbd_device, config);
    if (r != EOK)
    {
        return qnx_err_to_libusb(r);
    }

    dpriv->selected_configuration = config;
    free(dpriv->config_descriptor);
    
    r = read_configuration(&dpriv->config_descriptor, handle->dev, config);
    /* TODO: handle error conditions */

    return 0;
}

/* Set the alternate setting for an interface.
 *
 * You will only ever be asked to set the alternate setting for an
 * interface which was successfully claimed earlier.
 *
 * It's OK for this function to block.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if the alternate setting does not exist
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_set_interface_altsetting(struct libusb_device_handle *handle,
                                        int iface, int altsetting)
{
    int r;
    struct libusb_config_descriptor *config;
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(handle);

    /* Does this interface exist? */
    r = libusb_get_active_config_descriptor(handle->dev, &config);
    if (r != 0)
    {
        return r;
    }
    usbi_dbg("config->bNumInterfaces = %d, iface = %d", config->bNumInterfaces, iface);
    int iface_found = iface < config->bNumInterfaces;
    free(config);
    if (!iface_found)
    {
        return LIBUSB_ERROR_NOT_FOUND;
    }

    struct nto_qnx_device_priv *dpriv = __device_priv(handle->dev);

    r = usbd_select_interface(dpriv->usbd_device, iface, altsetting);

    if (r != EOK) 
    {
        return qnx_err_to_libusb(r);
    }

    hpriv->alt = altsetting;

    /* TODO: ensure this is a libusb error and not an ERRNO */
    return r;
}

/* Clear a halt/stall condition on an endpoint.
 *
 * It's OK for this function to block.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if the endpoint does not exist
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_clear_halt(struct libusb_device_handle *handle, unsigned char endpoint)
{
    struct nto_qnx_device_priv *dpriv = __device_priv(handle->dev);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(handle);

    usbd_descriptors_t * ud;
    struct usbd_desc_node * usbd_dn;
    struct usbd_pipe *ep_pipe;

    int r;

    int configuration = dpriv->selected_configuration;
    if ( configuration == -1 ) 
    {
        usbi_dbg("Didn't know what configuration to use,"
                 " sending out control transfer to figure it out.");
        /* if we haven't explicitly selected a configuration, find out
           what configuration the device is using */
        libusb_get_configuration(handle, &configuration);
        dpriv->selected_configuration = configuration;
    }

    int ifno = hpriv->interface_number;
    if ( ifno == -1 )
    {
        usbi_dbg("Didn't know the interface number,"
                 " but it should have been claimed. Trying to DWYM with interface 0.");
        /* well an interface SHOULD have been selected but just
           incase we will assume interface 0 */
        ifno = 0;
    }

    int alt = hpriv->alt;
    if ( alt == -1 )
    {
        usbi_dbg("Didn't know what alt setting to use,"
                 " but it should have been set by select_interface. Trying to DWYM with alt 0.");
        /* if no explicit altsetting, assume 0 */
        alt = 0;
    }

    ud = (usbd_descriptors_t *) 
        usbd_endpoint_descriptor(dpriv->usbd_device,
                                 configuration,
                                 ifno,
                                 alt,
                                 endpoint,
                                 &usbd_dn);

    if (ud == 0) {
        /* TODO: support this kind of error */
        usbi_err(HANDLE_CTX(handle), "usbd_endpoint_descriptor failed");
        return LIBUSB_ERROR_NO_MEM;
    }

    r = usbd_open_pipe(dpriv->usbd_device, ud, &ep_pipe);
    if (r != EOK) {
        /* TODO: Support this kind of error */
        usbi_err(HANDLE_CTX(handle), "usbd_open_pipe failed");
        return qnx_err_to_libusb(r);
    }

    /* Clear the stall/halt */
    usbd_reset_pipe(ep_pipe);
    usbd_close_pipe(ep_pipe);

    return r;
}

static int op_clock_gettime(int clk_id, struct timespec *tp)
{
	switch (clk_id) {
	case USBI_CLOCK_MONOTONIC:
		return clock_gettime(CLOCK_MONOTONIC, tp);
	case USBI_CLOCK_REALTIME:
		return clock_gettime(CLOCK_REALTIME, tp);
	default:
		return LIBUSB_ERROR_INVALID_PARAM;
  }
}


/* QNX specific function for canceling control transfers */
static int cancel_control_transfer(struct usbi_transfer *itransfer)
{
    int r;
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(transfer->dev_handle);
    
    
    usbi_info (ITRANSFER_CTX (itransfer),
               "WARNING: aborting ALL transactions on endpoint %x",
               transfer->endpoint);
    /* TODO: add slog entry for more consistent error reporting */
    r = usbd_abort_pipe(hpriv->control_pipe);
    if (r != EOK)
    {
        return LIBUSB_ERROR_IO;
    }
    return LIBUSB_SUCCESS;
}

static int cancel_transfers(struct usbi_transfer *itransfer)
{
    int r;
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    /* struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(transfer->dev_handle); */
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);

    usbi_info (ITRANSFER_CTX(itransfer),
               "WARNING: aborting ALL transactions on endpoint %x",
               transfer->endpoint);
    r = usbd_abort_pipe(tpriv->transfer_pipe);
    if (r != EOK)
    {
        return LIBUSB_ERROR_IO;
    }
    return LIBUSB_SUCCESS;
}

/* Cancel a previously submitted transfer.
 *
 * This function must not block. The transfer cancellation must complete
 * later, resulting in a call to usbi_handle_transfer_cancellation()
 * from the context of handle_events.
 */
static int op_cancel_transfer(struct usbi_transfer *itransfer)
{
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    
    switch (transfer->type) {
    case LIBUSB_TRANSFER_TYPE_CONTROL:
        return cancel_control_transfer(itransfer);
    case LIBUSB_TRANSFER_TYPE_BULK:
    case LIBUSB_TRANSFER_TYPE_INTERRUPT:
    case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
        return cancel_transfers (itransfer);
    default:
        usbi_err (TRANSFER_CTX(transfer), "unknown endpoint type %d", transfer->type);
        return LIBUSB_ERROR_INVALID_PARAM;
    }
}

/* Perform a USB port reset to reinitialize a device.
 *
 * If possible, the handle should still be usable after the reset
 * completes, assuming that the device descriptors did not change during
 * reset and all previous interface state can be restored.
 *
 * If something changes, or you cannot easily locate/verify the resetted
 * device, return LIBUSB_ERROR_NOT_FOUND. This prompts the application
 * to close the old handle and re-enumerate the device.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if re-enumeration is required, or if the device
 *   has been disconnected since it was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_reset_device(struct libusb_device_handle *handle)
{
    struct nto_qnx_device_priv * dpriv = __device_priv(handle->dev);
    int r;
    
    r = usbd_reset_device(dpriv->usbd_device);
    if (r != EOK) {
        usbi_err(HANDLE_CTX(handle), "failed to reset device, %s", strerror(r));
        return qnx_err_to_libusb(r);
    }

    /* TODO: is there actually any cleanup necessary here? */

    /* for now we will return LIBUSB_ERROR_NOT_FOUND so that the
       application knows to reconnect to the device */
    return LIBUSB_ERROR_NOT_FOUND;
}

/* Determine if a kernel driver is active on an interface. Optional.
 *
 * The presence of a kernel driver on an interface indicates that any
 * calls to claim_interface would fail with the LIBUSB_ERROR_BUSY code.
 *
 * Return:
 * - 0 if no driver is active
 * - 1 if a driver is active
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_kernel_driver_active(struct libusb_device_handle *handle, int interface)
{
    return LIBUSB_ERROR_NOT_SUPPORTED;
}
	
/* Detach a kernel driver from an interface. Optional.
 *
 * After detaching a kernel driver, the interface should be available
 * for claim.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if no kernel driver was active
 * - LIBUSB_ERROR_INVALID_PARAM if the interface does not exist
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - another LIBUSB_ERROR code on other failure
 */
static int op_detach_kernel_driver(struct libusb_device_handle *handle, int interface)
{
    return LIBUSB_ERROR_NOT_SUPPORTED;
}

/* Attach a kernel driver to an interface. Optional.
 *
 * Reattach a kernel driver to the device.
 *
 * Return:
 * - 0 on success
 * - LIBUSB_ERROR_NOT_FOUND if no kernel driver was active
 * - LIBUSB_ERROR_INVALID_PARAM if the interface does not exist
 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
 *   was opened
 * - LIBUSB_ERROR_BUSY if a program or driver has claimed the interface,
 *   preventing reattachment
 * - another LIBUSB_ERROR code on other failure
 */
static int op_attach_kernel_driver(struct libusb_device_handle *handle, int interface)
{
    return LIBUSB_ERROR_NOT_SUPPORTED;
}

/* Destroy a device. Optional.
 *
 * This function is called when the last reference to a device is
 * destroyed. It should free any resources allocated in the get_device_list
 * path.
 */
static void op_destroy_device(struct libusb_device *dev)
{
    struct nto_qnx_device_priv *dpriv = __device_priv(dev);
    int status;
    
    if (dpriv->dev_descriptor)
    {
        usbi_dbg("trying to free dev_descriptor...");
        free(dpriv->dev_descriptor);
    }

    if (dpriv->config_descriptor)
    {
        usbi_dbg("trying to free config_descriptor...");
        free(dpriv->config_descriptor);
    }

    if (dpriv->usbd_device)
    {
        status = usbd_detach(dpriv->usbd_device);
        if (status != EOK)
        {
            usbi_err(DEVICE_CTX(dev), "usbd_detach() failed");
        }
    }

    usbi_info(DEVICE_CTX(dev), "All done!");
}

/* Clear a transfer as if it has completed or cancelled, but do not
 * report any completion/cancellation to the library. You should free
 * all private data from the transfer as if you were just about to report
 * completion or cancellation.
 *
 * This function might seem a bit out of place. It is used when libusb
 * detects a disconnected device - it calls this function for all pending
 * transfers before reporting completion (with the disconnect code) to
 * the user. Maybe we can improve upon this internal interface in future.
 */
static void op_clear_transfer_priv(struct usbi_transfer *itransfer)
{
    struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
    struct nto_qnx_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
    struct nto_qnx_device_handle_priv *hpriv = __device_handle_priv(transfer->dev_handle);

    if (tpriv->urb) usbd_free_urb(tpriv->urb);
    if (tpriv->internal_buffer) usbd_free(tpriv->internal_buffer);
    
    if (!(tpriv->transfer_pipe == hpriv->control_pipe))
    {
        usbi_info(ITRANSFER_CTX(itransfer), "Yes we need to close the pipe");
        usbd_close_pipe(tpriv->transfer_pipe);
    }
}

/* Deinitialization. Optional. This function should destroy anything
 * that was set up by init.
 *
 * This function is called when the user deinitializes the library.
 */
static void op_exit(void)
{
    int status;
    libusb_context *ctx = NULL; /* NULL so that we can get the default context */

    /* Get the default context */
    USBI_GET_CONTEXT(ctx);
    usbi_info(ctx, "About to disconnect both global connections");

    if ((status = usbd_disconnect(global_connection)) != EOK)
    {
        usbi_err(ctx, "Failed to disconnect from the global read/write connection");
    }

    if ((status = usbd_disconnect(global_ro_connection)) != EOK)
    {
        usbi_err(ctx, "Failed to disconnect from the global read-only connection");
    }

    pthread_mutex_unlock(&init_mutex);
}

const struct usbi_os_backend nto_qnx_usbfs_backend = {
	.name = "QNX NTO usbfs",
	.init = op_init,
	.exit = op_exit,
	.get_device_list = op_get_device_list,
	.get_device_descriptor = op_get_device_descriptor,
	.get_active_config_descriptor = op_get_active_config_descriptor,
	.get_config_descriptor = op_get_config_descriptor,

	.open = op_open,
	.close = op_close,
	.get_configuration = op_get_configuration,
	.set_configuration = op_set_configuration,
	.claim_interface = op_claim_interface,
	.release_interface = op_release_interface, 

	.set_interface_altsetting = op_set_interface_altsetting, 
	.clear_halt = op_clear_halt, 
	.reset_device = op_reset_device, 

	.kernel_driver_active = op_kernel_driver_active, 
	.detach_kernel_driver = op_detach_kernel_driver, 
	.attach_kernel_driver = op_attach_kernel_driver, 

	.destroy_device = op_destroy_device,

	.submit_transfer = op_submit_transfer, 
	.cancel_transfer = op_cancel_transfer, 
	.clear_transfer_priv = op_clear_transfer_priv, 

	.handle_events = op_handle_events, 

	.clock_gettime = op_clock_gettime, 

    /* Not sure if we can do this, depends on whether or not
       CLOCK_MONOTONIC is an fd or not */
    /* #ifdef USBI_TIMERFD_AVAILABLE */
    /* 	.get_timerfd_clockid = NULL,  */
    /* #endif */

	.device_priv_size = sizeof(struct nto_qnx_device_priv),
	.device_handle_priv_size = sizeof(struct nto_qnx_device_handle_priv),
	.transfer_priv_size = sizeof(struct nto_qnx_transfer_priv),
	.add_iso_packet_size = 0

};

