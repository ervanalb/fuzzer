#include <stdlib.h>

#include "usbd_class.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include <string.h>
#include "hal.h"
#include "main.h"
#include "program.h"

#define MAX_PACKET_SIZE 64 

#define USB_REQ_IN 0x80

static int need_zlp = 0;
static int packet_sent = 0;
static int rx_open = 0;
static uint8_t tx_buf[MAX_PACKET_SIZE];
static uint8_t rx_buf[MAX_PACKET_SIZE];
static uint8_t bRequest;
static uint8_t wValue;
static uint8_t initialized = 0;

static uint8_t init_cb(void* pdev, uint8_t cfgidx) {
    DCD_PMA_Config(pdev, IN_EP, USB_SNG_BUF, BULK_IN_TX_ADDRESS);
    DCD_PMA_Config(pdev, OUT_EP, USB_SNG_BUF, BULK_OUT_RX_ADDRESS);

    // Open EPs
    DCD_EP_Open(pdev, IN_EP, MAX_PACKET_SIZE, USB_EP_BULK);
    DCD_EP_Open(pdev, OUT_EP, MAX_PACKET_SIZE, USB_EP_BULK);

    initialized = 1;

    return USBD_OK;
}

static uint8_t deinit_cb(void* pdev, uint8_t cfgidx) {
    /* Close EP IN */
    DCD_EP_Close(pdev, IN_EP);
    DCD_EP_Close(pdev, OUT_EP);

    return USBD_OK;
}

static uint8_t setup_cb(void* pdev, USB_SETUP_REQ* req) {
    // Only accept vendor requests
    if((req->bmRequest & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_VENDOR) {
        USBD_CtlError(pdev, req);
        return USBD_FAIL;
    }

    bRequest = req->bRequest;
    wValue = req->wValue;

    switch(req->bRequest) {
        case REQUEST_STREAM:
            if(req->bmRequest & USB_REQ_IN) {
                return USBD_FAIL;
            }
            if(req->wValue) {
                hal_stream_enable();
            } else {
                hal_stream_disable();
            }
            return USBD_OK;
        case REQUEST_LOAD_PROGRAM:
            if(req->bmRequest & USB_REQ_IN) {
                return USBD_FAIL;
            }
            if((&_suser_program) + req->wLength > (&_euser_program)) {
                return USBD_FAIL;
            }
            USBD_CtlPrepareRx(pdev, &_suser_program, req->wLength);
            return USBD_OK;
        default:
            return USBD_FAIL;
    }
}

static uint8_t ctl_rx_cb(void *pdev) {
    switch(bRequest) {
        case REQUEST_LOAD_PROGRAM:
            program_loaded();
            return USBD_OK;
        default:
            return USBD_FAIL;
    }
}

const uint8_t config_descriptor[] = {
    0x09,   /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE ,   /* bDescriptorType: Configuration */
    0x20,   /* wTotalLength (LSB) */
    0x00,   /* wTotalLength (MSB) */
    0x01,   /* bNumberInterfaces: 1 interface */
    0x01,   /* bConfigurationValue */
    0x00,   /* iConfiguration: Index of string descriptor for this config */
    0x80,   /* bmAttributes: bus powered */
    0x32,   /* bMaxPower: 100 mA */

    0x09,   /* bLength: interface descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType: Interface */
    0x00, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x02, /* bNumEndpoints: one endpoint */
    0xFF, /* bInterfaceClass: user's interface for vendor class */
    0x00, /* bInterfaceSubClass : */
    0x00, /* nInterfaceProtocol : None */
    0x05, /* iInterface: */

    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
    IN_EP,                             /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(MAX_PACKET_SIZE),      /* wMaxPacketSize: */
    HIBYTE(MAX_PACKET_SIZE),
    0x00,                               /* bInterval: ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
    OUT_EP,                            /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(MAX_PACKET_SIZE),      /* wMaxPacketSize: */
    HIBYTE(MAX_PACKET_SIZE),
    0x00                               /* bInterval: ignore for Bulk transfer */
};

static uint8_t* config_cb(uint8_t speed, uint16_t* length) {
    *length = sizeof(config_descriptor);
    return (uint8_t*)config_descriptor;
}

static void try_tx(void* pdev) {
/*
      if(packet_sent) return;

      int samples_available = hal_usb_available();

      if(samples_available >= MAX_PACKET_SIZE) {
            hal_stream_input(tx_buf, MAX_PACKET_SIZE);
            need_zlp = 1;
            DCD_EP_Tx(pdev, IN_EP, (uint8_t*)tx_buf, MAX_PACKET_SIZE);
            packet_sent = 1;
      } else if(hal_stream_input_enabled == 0 && (samples_available > 0 || need_zlp)) {
          hal_stream_input(tx_buf, samples_available);
          need_zlp = 0;
          DCD_EP_Tx(pdev, IN_EP, (uint8_t*)tx_buf, samples_available);
          packet_sent = 1;
      }
*/
}

static void try_rx(void *pdev) {
/*
    if(rx_open) return;

    uint16_t free_space = hal_stream_output_space();

    // If 64 bytes of space are free in TX buffer, open the EP
    if(free_space >= MAX_PACKET_SIZE) {
        DCD_EP_PrepareRx(pdev, OUT_EP, (uint8_t*)rx_buf, MAX_PACKET_SIZE);
        rx_open = 1;
    }
*/
}

static uint8_t tx_cb(void *pdev, uint8_t epnum) {
    // If 64 bytes available for read, send them out
    packet_sent = 0;
    try_tx(pdev);

    return USBD_OK;
}

static uint8_t rx_cb(void *pdev, uint8_t epnum) {
/*
    // Take the newly received data and put it in the write stream
    uint16_t length = ((USB_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;
    hal_stream_output(rx_buf, length);
    rx_open = 0;
    try_rx(pdev);
*/
    return USBD_OK;
}

static uint8_t sof_cb(void *pdev) {
    if(!initialized) return USBD_OK;

    try_tx(pdev);
    try_rx(pdev);

    return USBD_OK;
}

USBD_Class_cb_TypeDef  USBD_custom_cb = {
    init_cb,
    deinit_cb,
    setup_cb,
    NULL, /*EP0_TxSent*/
    ctl_rx_cb, /*EP0_RxReady*/
    tx_cb, /*DataIn*/
    rx_cb, /*DataOut*/
    sof_cb, /*SOF */
    config_cb,
};
