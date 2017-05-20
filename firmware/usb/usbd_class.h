#include "usbd_core.h"

#define USB_REQ_TYPE_VENDOR 0x40

extern USBD_Class_cb_TypeDef USBD_custom_cb;

#define REQUEST_STREAM_INPUT   0x20
#define REQUEST_STREAM_OUTPUT  0x21
#define REQUEST_CONFIGURE_PIN  0x22
