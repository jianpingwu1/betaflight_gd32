

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>

#include "platform.h"

#include "build/atomic.h"

#include "usbd_cdc_vcp.h"
#include "gd32f4xx.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#pragma     data_alignment = 4
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN usb_core_driver USB_OTG_dev __ALIGN_END;

LINE_CODING g_lc;

extern __IO uint8_t USB_Tx_State;
__IO uint32_t bDeviceState = UNCONNECTED; /* USB device status */

/* These are external variables imported from CDC core to be used for IN transfer management. */

/* This is the buffer for data received from the MCU to APP (i.e. MCU TX, APP RX) */
extern uint8_t APP_Rx_Buffer[APP_RX_DATA_SIZE];
extern volatile uint32_t APP_Rx_ptr_out;
/* Increment this buffer position or roll it back to
 start address when writing received data
 in the buffer APP_Rx_Buffer. */
extern volatile uint32_t APP_Rx_ptr_in;

/*
    APP TX is the circular buffer for data that is transmitted from the APP (host)
    to the USB device (flight controller).
*/
static uint8_t APP_Tx_Buffer[APP_TX_DATA_SIZE];
static uint32_t APP_Tx_ptr_out = 0;
static uint32_t APP_Tx_ptr_in = 0;

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx(const uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len);

void (*ctrlLineStateCb)(void* context, uint16_t ctrlLineState);
void *ctrlLineStateCbContext;
void (*baudRateCb)(void *context, uint32_t baud);
void *baudRateCbContext;

CDC_IF_Prop_TypeDef VCP_fops = {VCP_Init, VCP_DeInit, VCP_Ctrl, VCP_DataTx, VCP_DataRx };

/* Private functions ---------------------------------------------------------*/
/*!
    \brief      initialize the USB CDC virtual COM port layer
    \param[in]  none
    \param[out] none
    \retval     USBD_OK always when initialization succeeds
*/
static uint16_t VCP_Init(void)
{
    bDeviceState = CONFIGURED;
    ctrlLineStateCb = NULL;
    baudRateCb = NULL;
    return USBD_OK;
}

/*!
    \brief      de-initialize the USB CDC virtual COM port layer
    \param[in]  none
    \param[out] none
    \retval     USBD_OK always when de-initialization completes
*/
static uint16_t VCP_DeInit(void)
{
    bDeviceState = UNCONNECTED;
    return USBD_OK;
}

/*!
    \brief      copy line coding structure
    \param[in]  plc1 source line coding
    \param[out] plc2 destination line coding
    \retval     none
*/
static void ust_cpy(LINE_CODING* plc2, const LINE_CODING* plc1)
{
   plc2->bitrate    = plc1->bitrate;
   plc2->format     = plc1->format;
   plc2->paritytype = plc1->paritytype;
   plc2->datatype   = plc1->datatype;
}

/*!
    \brief      handle CDC class control requests
    \param[in]  Cmd request command code
    \param[in]  Buf pointer to command parameter buffer (also used as output for GET requests)
    \param[in]  Len length of buffer in bytes
    \param[out] none
    \retval     USBD_OK always (errors are not differentiated here)
*/
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
    LINE_CODING* plc = (LINE_CODING*)Buf;

    // assert_param(Len>=sizeof(LINE_CODING));

    switch (Cmd) {
       /* Not  needed for this driver, AT modem commands */
      case SEND_ENCAPSULATED_COMMAND:
      case GET_ENCAPSULATED_RESPONSE:
         break;

      // Not needed for this driver
      case SET_COMM_FEATURE:
      case GET_COMM_FEATURE:
      case CLEAR_COMM_FEATURE:
         break;

      //Note - hw flow control on UART 1-3 and 6 only
      case SET_LINE_CODING:
         // If a callback is provided, tell the upper driver of changes in baud rate
         if (plc && (Len == sizeof(*plc))) {
             if (baudRateCb) {
                 baudRateCb(baudRateCbContext, plc->bitrate);
             }
             ust_cpy(&g_lc, plc);           //Copy into structure to save for later
         }
         break;

      case GET_LINE_CODING:
         if (plc && (Len == sizeof(*plc))) {
             ust_cpy(plc, &g_lc);
         }
         break;

      case SET_CONTROL_LINE_STATE:
         // If a callback is provided, tell the upper driver of changes in DTR/RTS state
         if (plc && (Len == sizeof(uint16_t))) {
             if (ctrlLineStateCb) {
                 ctrlLineStateCb(ctrlLineStateCbContext, *((uint16_t *)Buf));
             }
         }
         break;

      case SEND_BREAK:
         /* Not  needed for this driver */
         break;

      default:
         break;
    }

    return USBD_OK;
}

/*!
    \brief      send data from device to host over CDC (wrapper)
    \param[in]  ptrBuffer pointer to data to transmit
    \param[in]  sendLength number of bytes to send
    \param[out] none
    \retval     number of bytes scheduled for transmission
*/
uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength)
{
    VCP_DataTx(ptrBuffer, sendLength);
    return sendLength;
}

uint32_t CDC_Send_FreeBytes(void)
{
    return APP_RX_DATA_SIZE - CDC_Receive_BytesAvailable();
}

/*!
    \brief      queue CDC data to be sent to host over USB
    \param[in]  Buf pointer to data buffer
    \param[in]  Len number of bytes to send
    \param[out] none
    \retval     USBD_OK when accepted
*/
static uint16_t VCP_DataTx(const uint8_t* Buf, uint32_t Len)
{
    /*
        make sure that any paragraph end frame is not in play
        could just check for: USB_CDC_ZLP, but better to be safe
        and wait for any existing transmission to complete.
    */
    while (USB_Tx_State != 0);

    for (uint32_t i = 0; i < Len; i++) {
        // Stall if the ring buffer is full
        while (((APP_Rx_ptr_in + 1) % APP_RX_DATA_SIZE) == APP_Rx_ptr_out) {
            delay(1);
        }

        APP_Rx_Buffer[APP_Rx_ptr_in] = Buf[i];
        APP_Rx_ptr_in = (APP_Rx_ptr_in + 1) % APP_RX_DATA_SIZE;
    }

    return USBD_OK;
}

/*!
    \brief      read data previously received from host into application buffer
    \param[out] recvBuf destination buffer
    \param[in]  len maximum number of bytes to read
    \retval     number of bytes actually copied
*/
uint32_t CDC_Receive_DATA(uint8_t* recvBuf, uint32_t len)
{
    uint32_t count = 0;

    while (APP_Tx_ptr_out != APP_Tx_ptr_in && (count < len)) {
        recvBuf[count] = APP_Tx_Buffer[APP_Tx_ptr_out];
        APP_Tx_ptr_out = (APP_Tx_ptr_out + 1) % APP_TX_DATA_SIZE;
        count++;
    }
    return count;
}

uint32_t CDC_Receive_BytesAvailable(void)
{
    /* return the bytes available in the receive circular buffer */
    return (APP_Tx_ptr_in + APP_TX_DATA_SIZE - APP_Tx_ptr_out) % APP_TX_DATA_SIZE;
}

/*!
    \brief      receive data from USB OUT endpoint into CDC circular buffer
    \param[in]  Buf pointer to received data
    \param[in]  Len number of bytes received
    \param[out] none
    \retval     USBD_OK if stored; USBD_FAIL if buffer overflow would occur
*/
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len)
{
    if (CDC_Receive_BytesAvailable() + Len > APP_TX_DATA_SIZE) {
        return USBD_FAIL;
    }

    for (uint32_t i = 0; i < Len; i++) {
        APP_Tx_Buffer[APP_Tx_ptr_in] = Buf[i];
        APP_Tx_ptr_in = (APP_Tx_ptr_in + 1) % APP_TX_DATA_SIZE;
    }

    return USBD_OK;
}

/*!
    \brief      check if USB device state is configured
    \param[in]  none
    \param[out] none
    \retval     non-zero if configured, zero otherwise
*/
uint8_t usbIsConfigured(void)
{
    return (bDeviceState == CONFIGURED);
}

/*!
    \brief      check if USB device cable/session is connected
    \param[in]  none
    \param[out] none
    \retval     non-zero if connected (not UNCONNECTED)
*/
uint8_t usbIsConnected(void)
{
    return (bDeviceState != UNCONNECTED);
}

/*!
    \brief      get current configured CDC line coding baud rate
    \param[in]  none
    \param[out] none
    \retval     baud rate in bps
*/
uint32_t CDC_BaudRate(void)
{
    return g_lc.bitrate;
}

/*!
    \brief      register callback invoked on baud rate change
    \param[in]  cb callback function pointer
    \param[in]  context user context passed to callback
    \param[out] none
    \retval     none
*/
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context)
{
    baudRateCbContext = context;
    baudRateCb = cb;
}

/*!
    \brief      register callback invoked on control line (DTR/RTS) state change
    \param[in]  cb callback function pointer
    \param[in]  context user context passed to callback
    \param[out] none
    \retval     none
*/
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}
