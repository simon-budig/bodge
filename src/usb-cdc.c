/*****************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ****************************************************************************/

#include <stdio.h>
#include "CH58x_common.h"

#define dg_log printf

#define THIS_ENDP0_SIZE         64
#define MAX_PACKET_SIZE         64

/* Device Descriptor */
const uint8_t TAB_USB_CDC_DEV_DES[] =
{
  0x12,        // bLength
  0x01,        // bDescriptorType
  0x10, 0x01,  // bcdUSB
  0x02,        // bDeviceClass
  0x00,        // bDeviceSubClass
  0x00,        // bDeviceProtocol
  0x40,        // bMaxPacketSize0
  0x86, 0x1a,  // idVendor
  0x40, 0x80,  // idProduct
  0x00, 0x30,  // bcdDevice
  0x01,        // iManufacturer
  0x02,        // iProduct
  0x03,        // iSeriial
  0x01         // bNumConfigurations
};

/* Configuration Descriptor */
const uint8_t TAB_USB_CDC_CFG_DES[] =
{
  0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x30,

  // Interface Descriptor
  0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

  0x05, 0x24, 0x00, 0x10, 0x01,   // CDC Header
  0x04, 0x24, 0x02, 0x02,         // CDC ACM
  0x05, 0x24, 0x06, 0x00, 0x01,   // CDC Union
  0x05, 0x24, 0x01, 0x01, 0x00,   // CDC Call Management

  0x07, 0x05, 0x84, 0x03, 0x08, 0x00, 0x01,      // EP4 IN (Interrupt)

  // Interface Descriptor
  0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,

  0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,      // EP1 OUT (Bulk)
  0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,      // EP1 IN  (Bulk)
};

/* Device Qualified Descriptor */
const uint8_t My_QueDescr[] = { 0x0a, 0x06, 0x00, 0x02, 0xff, 0x00, 0xff, 0x40, 0x01, 0x00 };

uint8_t TAB_CDC_LINE_CODING[]  =
{
  0x85,   /* baud rate*/
  0x20,
  0x00,
  0x00,
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* no. of bits 8*/
};

#define DEF_IC_PRG_VER     0x31

// Language descriptors
const uint8_t TAB_USB_LID_STR_DES[] = { 0x04, 0x03, 0x09, 0x04 };

const uint8_t USB_DEV_PARA_CDC_SERIAL_STR[]      = "WCH121212TS1";
const uint8_t USB_DEV_PARA_CDC_PRODUCT_STR[]     = "USB2.0 To Serial Port";
const uint8_t USB_DEV_PARA_CDC_MANUFACTURE_STR[] = "wch.cn";


typedef struct DevInfo
{
  uint8_t UsbConfig;      // USB configuration flags
  uint8_t UsbAddress;     // USB device address
  uint8_t gSetupReq;      // USB control transfer command code
  uint8_t gSetupLen;      // USB controls the transmission length
  uint8_t gUsbInterCfg;   // USB device interface configuration
  uint8_t gUsbFlag;       // Various operating flags for USB devices,
                          //     bit 0 = bus reset,
                          //     bit 1 = get device descriptor,
                          //     bit 2 = set address,
                          //     bit 3 = get configuration descriptor,
                          //     bit 4 = set configuration
} DevInfo_Parm;

/* Device information */
DevInfo_Parm  devinf;
uint8_t SetupReqCode, SetupLen;

/* Buffers for endpoint hardware and software operations */
__aligned (4) uint8_t  Ep0Buffer[MAX_PACKET_SIZE];     // Endpoint 0 Transmit and receive common Endpoint 4 OUT & IN

// The upload address of endpoint 4
__aligned (4) uint8_t  Ep1Buffer[MAX_PACKET_SIZE];     // IN
__aligned (4) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];   // OUT & IN
__aligned (4) uint8_t  Ep3Buffer[2*MAX_PACKET_SIZE];   // OUT & IN

// Line Code structure
typedef struct __PACKED _LINE_CODE
{
  uint32_t  BaudRate;   /* baud rate */
  uint8_t StopBits;     /* Stop bit counting, 0:1 stop bit, 1:1.5 stop bit, 2:2 stop bit */
  uint8_t ParityType;   /* Check digit, 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space */
  uint8_t DataBits;     /* Data bit count: 5, 6, 7, 8, 16 */
} LINE_CODE, *PLINE_CODE;

/* Data related to the two serial ports */
LINE_CODE Uart0Para;

/* Modem signal detection in vendor mode */
uint8_t UART0_RTS_Val = 0; // Output indicates that the DTE requests the DCE to send data
uint8_t UART0_DTR_Val = 0; // Output: The data terminal is ready
uint8_t UART0_OUT_Val = 0; // Custom modem signal (CH340 manual)

uint8_t UART0_DCD_Val    = 0;
uint8_t UART0_DCD_Change = 0;

uint8_t UART0_RI_Val    = 0;
uint8_t UART0_RI_Change = 0;

uint8_t UART0_DSR_Val    = 0;
uint8_t UART0_DSR_Change = 0;

uint8_t UART0_CTS_Val    = 0;
uint8_t UART0_CTS_Change = 0;

/* The serial port set by CDC */
uint8_t CDCSetSerIdx      = 0;
uint8_t CDCSer0ParaChange = 0;

typedef struct _USB_SETUP_REQ_ {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint8_t wIndexL;
    uint8_t wIndexH;
    uint8_t wLengthL;
    uint8_t wLengthH;
} USB_SETUP_REQ_t;

#define UsbSetupBuf     ((USB_SETUP_REQ_t *) Ep0Buffer) // USB_SETUP_REQ_t USB_SETUP_REQ_t

/* USB cache definition, all endpoints are defined */
/* Endpoint 1 -- IN status */
uint8_t Ep1DataINFlag = 0;

/* Endpoint 1 transmits data down */
uint8_t Ep1DataOUTFlag = 0;
uint8_t Ep1DataOUTLen  = 0;
__aligned (4) uint8_t Ep1OUTDataBuf[MAX_PACKET_SIZE];

/* Endpoint 2 -- IN Status */
uint8_t Ep2DataINFlag = 0;

/* Endpoint 2 uploads data down */
uint8_t Ep2DataOUTFlag = 0;
uint8_t Ep2DataOUTLen  = 0;
__aligned (4) uint8_t Ep2OUTDataBuf[MAX_PACKET_SIZE];

/* Save the status of USB interrupts - > change the operation mode to several groups */
#define USB_IRQ_FLAG_NUM     4

uint8_t usb_irq_w_idx = 0;
uint8_t usb_irq_r_idx = 0;

volatile uint8_t usb_irq_len[USB_IRQ_FLAG_NUM];
volatile uint8_t usb_irq_pid[USB_IRQ_FLAG_NUM];
volatile uint8_t usb_irq_flag[USB_IRQ_FLAG_NUM];

/* Endpoint 0 enumerates upload frame processing */
uint8_t ep0_send_buf[256];

/**********************************************************/
uint8_t DevConfig;
uint16_t SetupReqLen;
const uint8_t *pDescr;


/* Character Size */
#define HAL_UART_5_BITS_PER_CHAR   5
#define HAL_UART_6_BITS_PER_CHAR   6
#define HAL_UART_7_BITS_PER_CHAR   7
#define HAL_UART_8_BITS_PER_CHAR   8

/* Stop Bits */
#define HAL_UART_ONE_STOP_BIT      1
#define HAL_UART_TWO_STOP_BITS     2

/* Parity settings */
#define HAL_UART_NO_PARITY         0x00   // No checksum
#define HAL_UART_ODD_PARITY        0x01   // Odd check
#define HAL_UART_EVEN_PARITY       0x02   // Puppet checks
#define HAL_UART_MARK_PARITY       0x03   // Set 1 mark
#define HAL_UART_SPACE_PARITY      0x04   // Empty space


/* Endpoint state setting function */
void USBDevEPnINSetStatus (uint8_t ep_num, uint8_t type, uint8_t sta);

/* endpoints enumeration */
#define ENDP0    0x00
#define ENDP1    0x01
#define ENDP2    0x02
#define ENDP3    0x03
#define ENDP4    0x04

/* ENDP x Type */
#define ENDP_TYPE_IN    0x00      /* ENDP is IN Type */
#define ENDP_TYPE_OUT   0x01      /* ENDP is OUT Type */

/* Endpoint answer status definition */
/* OUT */
#define OUT_ACK       0
#define OUT_TIMOUT    1
#define OUT_NAK       2
#define OUT_STALL     3
/* IN */
#define IN_ACK        0
#define IN_NORSP      1
#define IN_NAK        2
#define IN_STALL      3

/* Definition of various flags for USB devices */
#define DEF_BIT_USB_RESET       0x01   /* Bus reset flag */
#define DEF_BIT_USB_DEV_DESC    0x02   /* Obtained device descriptor flags */
#define DEF_BIT_USB_ADDRESS     0x04   /* The address flag has been set */
#define DEF_BIT_USB_CFG_DESC    0x08   /* Obtained the configuration descriptor flag */
#define DEF_BIT_USB_SET_CFG     0x10   /* The configured value flag has been set */
#define DEF_BIT_USB_WAKE        0x20   /* Wake on USB flag */
#define DEF_BIT_USB_SUPD        0x40   /* USB bus hang flag */
#define DEF_BIT_USB_HS          0x80   /* USB high-speed, full-speed logo */

/* Interrupt handlers */
__attribute__ ((interrupt ("WCH-Interrupt-fast")))
__attribute__ ((section (".highcode")))
void USB_IRQHandler (void)
{
  uint8_t   j;

  if (R8_USB_INT_FG & RB_UIF_TRANSFER)   // transmission complete
    {
      /* Except for the setup package processing */
      if ((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN)     // Non-idle
        {
          /* Write directly */
          usb_irq_flag[usb_irq_w_idx] = 1;
          usb_irq_pid[usb_irq_w_idx]  = R8_USB_INT_ST;  // & 0x3f;// (0x30 | 0x0f);
          usb_irq_len[usb_irq_w_idx]  = R8_USB_RX_LEN;

          switch (usb_irq_pid[usb_irq_w_idx] & 0x3f)    // Analyze the current endpoint
            {
              case (UIS_TOKEN_OUT | 2):
                if (R8_USB_INT_FG & RB_U_TOG_OK)
                  {
                    R8_UEP2_CTRL ^= RB_UEP_R_TOG;
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xf3) | 0x08; // OUT_NAK
                    /* Save the data */
                    for (j = 0; j < (MAX_PACKET_SIZE/4); j++)
                      ((uint32_t *) Ep2OUTDataBuf)[j] = ((uint32_t *) Ep2Buffer)[j];
                  }
                else  // Out-of-sync packets are dropped
                  {
                    usb_irq_flag[usb_irq_w_idx] = 0;
                  }
                break;

              case (UIS_TOKEN_IN | 2):   // endpoint #2 Bulk endpoint upload completed
                R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
                R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xfc) | IN_NAK; // IN_NAK
                break;

              case (UIS_TOKEN_OUT | 1):
                if (R8_USB_INT_FG & RB_U_TOG_OK)
                  {
                    R8_UEP1_CTRL ^=  RB_UEP_R_TOG;
                    R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xf3) | 0x08; // OUT_NAK
                    /* Save the data */
                    for (j = 0; j < (MAX_PACKET_SIZE/4); j++)
                      ((uint32_t *) Ep1OUTDataBuf)[j] = ((uint32_t *) Ep1Buffer)[j];
                  }
                // Out-of-sync packets are dropped
                else
                  {
                    usb_irq_flag[usb_irq_w_idx] = 0;
                  }
                break;

              case (UIS_TOKEN_IN | 1):  // endpoint #1 Batch endpoint upload completed
                R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
                R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xfc) | IN_NAK; // IN_NAK
                break;

              case (UIS_TOKEN_OUT | 0):    // endpoint 0
                if (R8_USB_INT_FG & RB_U_TOG_OK)   // Out-of-sync packets are dropped
                  R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xf3) | 0x08; // OUT_NAK
                else
                  usb_irq_flag[usb_irq_w_idx] = 0;
                break;

              case (UIS_TOKEN_IN | 0):  // endpoint 0
                R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xfc) | IN_NAK; // IN_NAK
                break;
            }

          /* Switch to the next write address */
          if (usb_irq_flag[usb_irq_w_idx])
            {
              usb_irq_w_idx++;
              if (usb_irq_w_idx >= USB_IRQ_FLAG_NUM)
                usb_irq_w_idx = 0;
            }

          // clear transfer interrupt flag
          R8_USB_INT_FG = RB_UIF_TRANSFER;
        }

      /* setup package processing */
      if (R8_USB_INT_ST & RB_UIS_SETUP_ACT)
        {
          /* Convergence with the previous process - UIS_TOKEN_SETUP */
          /* Write directly */
          usb_irq_flag[usb_irq_w_idx] = 1;
          usb_irq_pid[usb_irq_w_idx]  = UIS_TOKEN_SETUP | 0;  // Keep the previous logo
          usb_irq_len[usb_irq_w_idx]  = 8;
          /* Switch to the next write address */
          usb_irq_w_idx++;
          if (usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;

          R8_USB_INT_FG = RB_UIF_TRANSFER;
        }
    }
}


uint8_t Ep4DataINFlag;
uint8_t Ep3DataINFlag;

uint8_t Ep1DataOUTLen ;
uint8_t Ep1DataOUTFlag ;

uint8_t Ep3DataOUTFlag = 0;
uint8_t Ep4DataOUTFlag = 0;


/* Class requests */
//  3.1 Requests---Abstract Control Model
#define DEF_SEND_ENCAPSULATED_COMMAND  0x00
#define DEF_GET_ENCAPSULATED_RESPONSE  0x01
#define DEF_SET_COMM_FEATURE           0x02
#define DEF_GET_COMM_FEATURE           0x03
#define DEF_CLEAR_COMM_FEATURE         0x04
#define DEF_SET_LINE_CODING            0x20   // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING            0x21   // This request allows the host to find out the currently configured line coding.
// #define DEF_SET_CTL_LINE_STE         0x22   // This request generates RS-232/V.24 style control signals.
#define DEF_SET_CONTROL_LINE_STATE     0x22
#define DEF_SEND_BREAK                 0x23

//  3.2 Notifications---Abstract Control Model
#define DEF_NETWORK_CONNECTION         0x00
#define DEF_RESPONSE_AVAILABLE         0x01
#define DEF_SERIAL_STATE               0x20


void USB_IRQProcessHandler (uint8_t **indata, uint8_t *indata_len)   /* USB interrupt service program */
{
  static uint8_t *pDescr;
  uint8_t   len;
  uint8_t   data_dir = 0;   // Data direction
  uint8_t   i;

  // for (i = 0; i < USB_IRQ_FLAG_NUM; i++)
    {
      i = usb_irq_r_idx;

      if (usb_irq_flag[i])
        {
          usb_irq_r_idx++;

          if (usb_irq_r_idx >= USB_IRQ_FLAG_NUM)
            usb_irq_r_idx = 0;

          switch (usb_irq_pid[i] & 0x3f)   // Analyze the action token and endpoint number
            {
              case UIS_TOKEN_IN | 4:  // endpoint #4 Batch endpoint upload completed
                 Ep4DataINFlag = ~0;
                 break;

              case UIS_TOKEN_IN | 3:  // endpoint #3 The bulk endpoint upload is complete
                Ep3DataINFlag = ~0;
                break;

              case UIS_TOKEN_OUT | 2:    // endpoint #2 The batch endpoint upload is complete
                dg_log ("usb_rec\n");
                len = usb_irq_len[i];

                // Ep2OUTDataBuf
                for (int i = 0; i < len; i++)
                  dg_log ("%02x  ", Ep2OUTDataBuf[i]);
                dg_log ("\n");

                // Data delivery of CH341
                Ep2DataOUTFlag = 1;
                Ep2DataOUTLen = len;
                PFIC_DisableIRQ (USB_IRQn);
                R8_UEP2_CTRL = R8_UEP2_CTRL & 0xf3; // OUT_ACK
                PFIC_EnableIRQ (USB_IRQn);
                break;

              case UIS_TOKEN_IN | 2:  // endpoint #2 Bulk endpoint upload completed
                Ep2DataINFlag = 1;
                break;

              case UIS_TOKEN_OUT | 1:    // endpoint #1 Batch endpoint upload is complete
                dg_log ("usb_rec\n");
                len = usb_irq_len[i];
                // Ep1OUTDataBuf
                for (int i = 0; i < len; i++)
                  dg_log ("%02x  ", Ep1OUTDataBuf[i]);
                dg_log ("\n");

                // Data delivery of CH341
                Ep1DataOUTFlag = 1;
                Ep1DataOUTLen = len;
                *indata = Ep1OUTDataBuf;
                *indata_len = len;
                PFIC_DisableIRQ (USB_IRQn);
                R8_UEP1_CTRL = R8_UEP1_CTRL & 0xf3; // OUT_ACK
                PFIC_EnableIRQ (USB_IRQn);
                break;

              case UIS_TOKEN_IN | 1:   // endpoint #1 Interrupt the endpoint upload is complete
                Ep1DataINFlag = 1;
                break;

              case UIS_TOKEN_SETUP | 0:    // endpoint #0 SETUP
                len = usb_irq_len[i];
                if (len == sizeof (USB_SETUP_REQ))
                  {
                    SetupLen = UsbSetupBuf->wLengthL;
                    if (UsbSetupBuf->wLengthH)
                      SetupLen = 0xff;

                    len = 0;          // The default is Succeeded and the upload is 0 length
                    SetupReqCode = UsbSetupBuf->bRequest;

                    /* Data direction */
                    data_dir = USB_REQ_TYP_OUT;
                    if (UsbSetupBuf->bRequestType & USB_REQ_TYP_IN)
                      data_dir = USB_REQ_TYP_IN;

                    // Standard Request
                    if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
                      {
                        switch (SetupReqCode)  // Request code
                          {
                            case USB_GET_DESCRIPTOR:  // Get a descriptor
                              switch (UsbSetupBuf->wValueH)
                                {
                                  case 1: // Device descriptor
                                    memcpy (ep0_send_buf,
                                           &TAB_USB_CDC_DEV_DES[0],
                                           sizeof (TAB_USB_CDC_DEV_DES));

                                    pDescr = ep0_send_buf;
                                    len = sizeof (TAB_USB_CDC_DEV_DES);
                                    break;

                                  case 2:  // Configure descriptors
                                    memcpy (ep0_send_buf,
                                           &TAB_USB_CDC_CFG_DES[0],
                                           sizeof (TAB_USB_CDC_CFG_DES));
                                    pDescr = ep0_send_buf;
                                    len = sizeof (TAB_USB_CDC_CFG_DES);
                                    break;

                                  case 3:  // String descriptors
                                    dg_log ("str %d\r\n", UsbSetupBuf->wValueL);
                                    dg_log ("str %d\r\n", UsbSetupBuf->wValueL);
                                    switch (UsbSetupBuf->wValueL)
                                      {
                                        case 0:  // Language descriptors
                                          pDescr = (uint8_t *) (&TAB_USB_LID_STR_DES[0]);
                                          len = sizeof (TAB_USB_LID_STR_DES);

                                          break;

                                        case 1:   // iManufacturer
                                        case 2:   // iProduct
                                        case 3:   // iSerialNumber
                                          {
                                            uint8_t ep0_str_len;
                                            uint8_t *p_send;
                                            uint8_t *manu_str;
                                            uint8_t tmp;

                                            /* Take the length */
                                            if (UsbSetupBuf->wValueL == 1)
                                              manu_str = (uint8_t *) USB_DEV_PARA_CDC_MANUFACTURE_STR;
                                            else if (UsbSetupBuf->wValueL == 2)
                                              manu_str = (uint8_t *) USB_DEV_PARA_CDC_PRODUCT_STR;
                                            else if (UsbSetupBuf->wValueL == 3)
                                              manu_str = (uint8_t *) USB_DEV_PARA_CDC_SERIAL_STR;

                                            ep0_str_len = (uint8_t) strlen ((char *) manu_str);
                                            p_send = ep0_send_buf;
                                            *p_send++ = ep0_str_len*2 + 2;
                                            *p_send++ = 0x03;
                                            for (tmp = 0; tmp < ep0_str_len; tmp++)
                                            {
                                              *p_send++ = manu_str[tmp];
                                              *p_send++ = 0x00;
                                            }

                                            pDescr = ep0_send_buf;
                                            len = ep0_send_buf[0];
                                          }
                                          break;

                                        default:
                                          len = 0xff;    // Unsupported descriptor types
                                          break;
                                      }
                                    break;

                                  case 6:  // Device Qualified Descriptor
                                    pDescr = (uint8_t *) (&My_QueDescr[0]);
                                    len = sizeof (My_QueDescr);
                                    break;

                                  default:
                                    len = 0xff;                                  // Unsupported descriptor types
                                    break;
                                }
                              if (SetupLen > len)
                                SetupLen = len;            // Limit the overall length

                              len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // The length of this transfer
                              memcpy (Ep0Buffer, pDescr, len);                 /* Load the feed */
                              SetupLen -= len;
                              pDescr += len;
                              break;

                            case USB_SET_ADDRESS:  // Set the address
                              dg_log ("SET_ADDRESS:%d\r\n", UsbSetupBuf->wValueL);
                              devinf.gUsbFlag |= DEF_BIT_USB_ADDRESS;
                              devinf.UsbAddress = UsbSetupBuf->wValueL;    // Staging USB device addresses
                              break;

                            case USB_GET_CONFIGURATION:
                              dg_log ("GET_CONFIGURATION\r\n");
                              Ep0Buffer[0] = devinf.UsbConfig;
                              if (SetupLen >= 1)
                                len = 1;
                              break;

                            case USB_SET_CONFIGURATION:
                              dg_log ("SET_CONFIGURATION\r\n");
                              devinf.gUsbFlag |= DEF_BIT_USB_SET_CFG;
                              devinf.UsbConfig = UsbSetupBuf->wValueL;
                              break;

                            case USB_CLEAR_FEATURE:
                              dg_log ("CLEAR_FEATURE\r\n");
                              len = 0;
                              /* Clear the device */
                              if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                                {
                                  R8_UEP1_CTRL = (R8_UEP1_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK;
                                  R8_UEP2_CTRL = (R8_UEP2_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK;
                                  R8_UEP3_CTRL = (R8_UEP3_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK;
                                  R8_UEP4_CTRL = (R8_UEP4_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK;

                                  // The state variable is reset
                                  Ep1DataINFlag = 1;
                                  Ep2DataINFlag = 1;
                                  Ep3DataINFlag = 1;
                                  Ep4DataINFlag = 1;

                                  Ep1DataOUTFlag = 0;
                                  Ep2DataOUTFlag = 0;
                                  Ep3DataOUTFlag = 0;
                                  Ep4DataOUTFlag = 0;
                                }
                              else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)  // endpoint
                                {
                                  switch (UsbSetupBuf->wIndexL)   // Judgment endpoint
                                    {
                                      case 0x84: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK; break;
                                      case 0x04: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~(RB_UEP_R_TOG | MASK_UEP_R_RES))) | UEP_R_RES_ACK; break;
                                      case 0x83: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK; break;
                                      case 0x03: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~(RB_UEP_R_TOG | MASK_UEP_R_RES))) | UEP_R_RES_ACK; break;
                                      case 0x82: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK; break;
                                      case 0x02: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~(RB_UEP_R_TOG | MASK_UEP_R_RES))) | UEP_R_RES_ACK; break;
                                      case 0x81: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~(RB_UEP_T_TOG | MASK_UEP_T_RES))) | UEP_T_RES_NAK; break;
                                      case 0x01: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~(RB_UEP_R_TOG | MASK_UEP_R_RES))) | UEP_R_RES_ACK; break;
                                      default: len = 0xff;  break;
                                    }
                                }
                              else
                                {
                                  len = 0xff;                                  // It's not that the endpoint isn't supported
                                }
                              break;

                            case USB_GET_INTERFACE:
                              dg_log ("GET_INTERFACE\r\n");
                              Ep0Buffer[0] = 0x00;
                              if (SetupLen >= 1)
                                len = 1;
                              break;

                            case USB_GET_STATUS:
                              dg_log ("GET_STATUS\r\n");
                              Ep0Buffer[0] = 0x00;
                              Ep0Buffer[1] = 0x00;
                              if (SetupLen >= 2)
                                len = 2;
                              else
                                len = SetupLen;
                              break;

                            default:
                              len = 0xff;                                       // The operation failed
                              break;
                          }
                      }
                    /* Class requests */
                    else if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS)
                      {
                        /* The host is downloaded */
                        if (data_dir == USB_REQ_TYP_OUT)
                          {
                            switch (SetupReqCode)  // Request code
                              {
                                case DEF_SET_LINE_CODING: /* SET_LINE_CODING */
                                  {
                                    uint8_t i;
                                    dg_log ("SET_LINE_CODING\r\n");
                                    for (i = 0; i < 8; i++)
                                      {
                                        dg_log ("%02x ", Ep0Buffer[i]);
                                      }
                                    dg_log ("\r\n");
                                    if (Ep0Buffer[ 4 ] == 0x00)
                                      {
                                        CDCSetSerIdx = 0;
                                        len = 0x00;
                                      }
                                    else if (Ep0Buffer[ 4 ] == 0x02)
                                     {
                                       CDCSetSerIdx = 1;
                                       len = 0x00;
                                     }
                                    else
                                      {
                                        len = 0xff;
                                      }
                                  }
                                  break;

                                case DEF_SET_CONTROL_LINE_STATE:  /* SET_CONTROL_LINE_STATE */
                                  {
                                    uint8_t  carrier_sta;
                                    uint8_t  present_sta;
                                    /* Line status */
                                    dg_log ("ctl %02x %02x\r\n", Ep0Buffer[2], Ep0Buffer[3]);
                                    carrier_sta = Ep0Buffer[2] & (1<<1);   // RTS status
                                    present_sta = Ep0Buffer[2] & (1<<0);   // DTR status
                                    len = 0;
                                  }
                                  break;

                                default:
                                  dg_log ("CDC ReqCode%x\r\n", SetupReqCode);
                                  len = 0xff;                                       // The operation failed
                                  break;
                              }
                          }
                        /* Device upload */
                        else
                          {
                            switch (SetupReqCode)  // Request code
                              {
                                case DEF_GET_LINE_CODING: /* GET_LINE_CODING */
                                  dg_log ("GET_LINE_CODING:%d\r\n", Ep0Buffer[ 4 ]);
                                  pDescr = Ep0Buffer;
                                  len = sizeof (LINE_CODE);
                                  ((PLINE_CODE) Ep0Buffer)->BaudRate   = Uart0Para.BaudRate;
                                  ((PLINE_CODE) Ep0Buffer)->StopBits   = Uart0Para.StopBits;
                                  ((PLINE_CODE) Ep0Buffer)->ParityType = Uart0Para.ParityType;
                                  ((PLINE_CODE) Ep0Buffer)->DataBits   = Uart0Para.DataBits;
                                  break;

                                case DEF_SERIAL_STATE:
                                  dg_log ("GET_SERIAL_STATE:%d\r\n", Ep0Buffer[ 4 ]);
                                  // SetupLen determines the overall length
                                  len = 2;
                                  CDCSetSerIdx = 0;
                                  Ep0Buffer[0] = 0;
                                  Ep0Buffer[1] = 0;
                                  break;

                                default:
                                  dg_log ("CDC ReqCode%x\r\n", SetupReqCode);
                                  len = 0xff;                                       // The operation failed
                                  break;
                              }
                          }
                      }

                    else len = 0xff;   /* fail */
                  }
                else
                  {
                    len = 0xff; // The length of the SETUP package is incorrect
                  }

                if (len == 0xff)  // The operation failed
                  {
                    SetupReqCode = 0xff;
                    PFIC_DisableIRQ (USB_IRQn);
                    R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  // STALL
                    PFIC_EnableIRQ (USB_IRQn);
                  }
                else if (len <= THIS_ENDP0_SIZE)  // A packet of 0 length is returned in the upload or status phase
                  {
                    if (SetupReqCode ==  USB_SET_ADDRESS)  // Set the address 0x05
                      {
////                      dg_log ("add in:%d\r\n", len);
                        PFIC_DisableIRQ (USB_IRQn);
                        R8_UEP0_T_LEN = len;
                        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
                        PFIC_EnableIRQ (USB_IRQn);
                      }
                    else if (SetupReqCode ==  USB_SET_CONFIGURATION)  // Set the configuration value 0x09
                      {
                        PFIC_DisableIRQ (USB_IRQn);
                        R8_UEP0_T_LEN = len;
                        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
                        PFIC_EnableIRQ (USB_IRQn);
                      }
                    else if (SetupReqCode ==  USB_GET_DESCRIPTOR)  // Get the descriptor 0x06
                      {
                        R8_UEP0_T_LEN = len;
                        PFIC_DisableIRQ (USB_IRQn);
                        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  // The default packet is DATA1
                        PFIC_EnableIRQ (USB_IRQn);
                      }
                    else if (SetupReqCode ==  DEF_SET_CONTROL_LINE_STATE)  // 0x22
                      {
                        PFIC_DisableIRQ (USB_IRQn);
                        R8_UEP0_T_LEN = len;
                        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
                        PFIC_EnableIRQ (USB_IRQn);
                      }
                    else if (SetupReqCode ==  USB_CLEAR_FEATURE)  // 0x01
                      {
                        PFIC_DisableIRQ (USB_IRQn);
                        R8_UEP0_T_LEN = len;
                        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
                        PFIC_EnableIRQ (USB_IRQn);
                      }
                    else
                      {
                        if (data_dir == USB_REQ_TYP_IN)   // Upload is currently required
                          {
                            PFIC_DisableIRQ (USB_IRQn);
                            R8_UEP0_T_LEN = len;
                            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
                            PFIC_EnableIRQ (USB_IRQn);
                          }
                        else                            // Downloading is currently required
                          {
                            PFIC_DisableIRQ (USB_IRQn);
                            R8_UEP0_T_LEN = len;
                            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;  // The default packet is DATA1
                            PFIC_EnableIRQ (USB_IRQn);
                          }
                      }
                }
              else  // Downloading data or others
                {
                  // Although it has not yet reached the state stage, it is necessary to preset and upload 0-length packets in advance to prevent the host from entering the state phase in advance
                  R8_UEP0_T_LEN = 0;
                  PFIC_DisableIRQ (USB_IRQn);
                  R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  // The default packet is DATA1
                  PFIC_EnableIRQ (USB_IRQn);
                }
                break;

              case UIS_TOKEN_IN | 0:      // endpoint #0 IN  UIS_TOKEN_IN
                switch (SetupReqCode)
                  {
                    /* Simple handling of SETUP commands */
                    case USB_GET_DESCRIPTOR:  // 0x06 Get a descriptor
                      len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // The length of this transfer
                      memcpy (Ep0Buffer, pDescr, len);                    /* Load the feed */
                      SetupLen -= len;
                      pDescr += len;

                      if (len)
                        {
                          R8_UEP0_T_LEN = len;
                          PFIC_DisableIRQ (USB_IRQn);
                          R8_UEP0_CTRL ^=  RB_UEP_T_TOG;
                          USBDevEPnINSetStatus (ENDP0, ENDP_TYPE_IN, IN_ACK);
                          PFIC_EnableIRQ (USB_IRQn);
                        }
                      else
                        {
                          R8_UEP0_T_LEN = len;
                          PFIC_DisableIRQ (USB_IRQn);
                          R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
                          PFIC_EnableIRQ (USB_IRQn);
                        }
                      break;

                    case USB_SET_ADDRESS:   // 0x05
                      R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | (devinf.UsbAddress);
                      PFIC_DisableIRQ (USB_IRQn);
                      R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
                      PFIC_EnableIRQ (USB_IRQn);
////                    dg_log ("add in deal\r\n");

                      break;

                    case DEF_GET_LINE_CODING:  // 0x21
                      PFIC_DisableIRQ (USB_IRQn);
                      R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
                      PFIC_EnableIRQ (USB_IRQn);

                      break;

                    case DEF_SET_LINE_CODING:   // 0x20
                      PFIC_DisableIRQ (USB_IRQn);
                      R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
                      PFIC_EnableIRQ (USB_IRQn);
                      break;

                    default:
                      R8_UEP0_T_LEN = 0; // The state phase completes an interrupt or forces the upload of 0-length packets to end the control transmission
                      PFIC_DisableIRQ (USB_IRQn);
                      R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
                      PFIC_EnableIRQ (USB_IRQn);

                      break;
                  }

                break;

              case UIS_TOKEN_OUT | 0:  // endpoint #0 OUT
                len = usb_irq_len[i];
                if (len)
                  {
                    switch (SetupReqCode)
                      {
                        /* Set the serial port */
                        case DEF_SET_LINE_CODING:
                          {
                            uint32_t set_bps;
                            uint8_t  data_bit;
                            uint8_t  stop_bit;
                            uint8_t  ver_bit;

                            memcpy (&set_bps, Ep0Buffer, 4);
                            stop_bit = Ep0Buffer[4];
                            ver_bit = Ep0Buffer[5];
                            data_bit = Ep0Buffer[6];

                            dg_log ("LINE_CODING %d %d %d %d %d\r\n", CDCSetSerIdx
                                                 , (int) set_bps
                                                 , data_bit
                                                 , stop_bit
                                                 , ver_bit);

                            Uart0Para.BaudRate = set_bps;
                            Uart0Para.StopBits = stop_bit;
                            Uart0Para.ParityType = ver_bit;
                            Uart0Para.DataBits = data_bit;
                            CDCSer0ParaChange = 1;

                            PFIC_DisableIRQ (USB_IRQn);
                            R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
                            PFIC_EnableIRQ (USB_IRQn);
                          }
                          break;

                        default:
                          PFIC_DisableIRQ (USB_IRQn);
                          R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
                          PFIC_EnableIRQ (USB_IRQn);
                          break;
                      }
                  }
                else
                  {
                    PFIC_DisableIRQ (USB_IRQn);
                    R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_NAK;
                    PFIC_EnableIRQ (USB_IRQn);
                  }
                break;

              default:
                break;
            }

          PFIC_DisableIRQ (USB_IRQn);
          usb_irq_flag[i] = 0;
          PFIC_EnableIRQ (USB_IRQn);
        }
    }

  if (R8_USB_INT_FG & RB_UIF_BUS_RST)  // USB bus reset
    {
      R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
      R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP3_CTRL = UEP_T_RES_NAK;
      R8_UEP4_CTRL = UEP_T_RES_NAK;

      R8_USB_DEV_AD = 0x00;
      devinf.UsbAddress = 0;

      R8_USB_INT_FG = RB_UIF_BUS_RST;       // Clear the break sign
    }
  else if (R8_USB_INT_FG & RB_UIF_SUSPEND)  // USB bus hang/wake complete
    {
      if (R8_USB_MIS_ST & RB_UMS_SUSPEND)   // Suspend
        {
          CDCSer0ParaChange = 1;

          Ep1DataINFlag = 0;
          Ep2DataINFlag = 0;
          Ep3DataINFlag = 0;
          Ep4DataINFlag = 0;

          Ep1DataOUTFlag = 0;
          Ep2DataOUTFlag = 0;
          Ep3DataOUTFlag = 0;
          Ep4DataOUTFlag = 0;
        }
      else                                   // awaken
        {
          Ep1DataINFlag = 1;
          Ep2DataINFlag = 1;
          Ep3DataINFlag = 1;
          Ep4DataINFlag = 1;

          Ep1DataOUTFlag = 0;
          Ep2DataOUTFlag = 0;
          Ep3DataOUTFlag = 0;
          Ep4DataOUTFlag = 0;
        }

        R8_USB_INT_FG = RB_UIF_SUSPEND;
    }
}


/*******************************************************************************
* Function Name  : USBDevEPnINSetStatus
* Description    : Endpoint state setting function
* Input          : ep_num: Endpoint number
*                  type: endpoint transport type
*                  sta: The state of the endpoint toggled
* Output         : None
* Return         : None
*******************************************************************************/
void USBDevEPnINSetStatus (uint8_t ep_num, uint8_t type, uint8_t sta)
{
  uint8_t *p_UEPn_CTRL;

  p_UEPn_CTRL = (uint8_t *) (USB_BASE_ADDR + 0x22 + ep_num * 4);

  if (type == ENDP_TYPE_IN)
    *((PUINT8V) p_UEPn_CTRL) = (*((PUINT8V) p_UEPn_CTRL) & (~(0x03))) | sta;
  else
    *((PUINT8V) p_UEPn_CTRL) = (*((PUINT8V) p_UEPn_CTRL) & (~(0x03<<2))) | (sta<<2);
}


/*******************************************************************************
* Function Name  : USBParaInit
* Description    : USB parameter initialization, cache and flags
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBParaInit (void)
{
  Ep1DataINFlag  = 1;
  Ep1DataOUTFlag = 0;
  Ep2DataINFlag  = 1;
  Ep2DataOUTFlag = 0;
  Ep3DataINFlag  = 1;
  Ep3DataOUTFlag = 0;
  Ep4DataINFlag  = 1;
  Ep4DataOUTFlag = 0;
}


/*******************************************************************************
* Function Name  : InitCDCDevice
* Description    : Initialize the CDC device
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitCDCDevice (void)
{
  /* Initialize the cache */
  USBParaInit ();

  R8_USB_CTRL = 0x00;         // Set the mode first

//  1. Endpoint Assignment:
//  Endpoint 0
//  Endpoint 1: IN and OUT (Data Interface)
//  Endpoint 2: IN and OUT (Data Interface)
//  Endpoint 3: IN (interface 23 combination, interrupt upload)
//  Endpoint 4: IN (interface 01 combination, interrupt upload)

  R8_UEP4_1_MOD = RB_UEP4_TX_EN|RB_UEP1_TX_EN|RB_UEP1_RX_EN;

  /* Single 64-Byte Receive Buffer (OUT), Single 64-Byte Send Buffer (IN) */
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_TX_EN;

  R16_UEP0_DMA = (uint16_t) (uint32_t) &Ep0Buffer[0];
  R16_UEP1_DMA = (uint16_t) (uint32_t) &Ep1Buffer[0];
  R16_UEP2_DMA = (uint16_t) (uint32_t) &Ep2Buffer[0];
  R16_UEP3_DMA = (uint16_t) (uint32_t) &Ep3Buffer[0];
  // R16_UEP4_DMA = (uint16_t) (uint32_t) &Ep2Buffer[0];

  /* ENDPOINT 0 STATE: OUT---ACK IN--NAK */
  R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;

  /* Endpoint 1 state: OUT--ACK IN--NAK auto-flip */
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* Endpoint 2 state: OUT--ACK IN--NAK auto-flip */
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* Endpoint 3 state: IN--NAK auto-flip */
  R8_UEP3_CTRL = UEP_T_RES_NAK;

  /* Endpoint 4 state: IN--NAK manual flip */
  R8_UEP4_CTRL = UEP_T_RES_NAK;

  /* Device address */
  R8_USB_DEV_AD = 0x00;

  // DP/DM pull-down resistors are disabled
  R8_UDEV_CTRL = RB_UD_PD_DIS;

  // Boot the USB device and DMA and automatically return the NAK
  // until the interrupt flag is cleared during the outage
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

  // Clear the break sign
  R8_USB_INT_FG = 0xff;

  // Unified program query?
  // Turn on interrupt, pend, transfer complete, bus reset
  // R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
  R8_USB_INT_EN = RB_UIE_TRANSFER ;
  PFIC_EnableIRQ (USB_IRQn);

  // Enable the USB port
  R8_UDEV_CTRL |= RB_UD_PORT_EN;

  devinf.UsbConfig = 0;
  devinf.UsbAddress = 0;
}


/*******************************************************************************
* Function Name  : InitUSBDefPara
* Description    : USB-related variable initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitUSBDefPara (void)
{
  uint8_t i;

  Uart0Para.BaudRate = 115200;
  Uart0Para.DataBits = HAL_UART_8_BITS_PER_CHAR;
  Uart0Para.ParityType = HAL_UART_NO_PARITY;
  Uart0Para.StopBits = HAL_UART_ONE_STOP_BIT;

  CDCSetSerIdx = 0;
  CDCSer0ParaChange = 0;

  UART0_DCD_Val = 0;
  UART0_RI_Val = 0;
  UART0_DSR_Val = 0;
  UART0_CTS_Val = 0;

  UART0_RTS_Val = 0; // Output indicates that the DTE requests the DCE to send data
  UART0_DTR_Val = 0; // Output: The data terminal is ready
  UART0_OUT_Val = 0; // Custom modem signal (CH340 manual)

  for (i = 0; i < USB_IRQ_FLAG_NUM; i++)
    usb_irq_flag[i] = 0;
}


/*******************************************************************************
* Function Name  : InitUSBDevice
* Description    : Initialize USB
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitUSBDevice (void)
{
  InitCDCDevice ();
}


/* Communications */
/*******************************************************************************
* Function Name  : SendUSBData
* Description    : Send data processing
* Input          : p_send_dat: A pointer to the data sent
*                  send_len: The status of the send
* Output         : None
* Return: The status of the send
*******************************************************************************/
uint8_t SendUSBData (uint8_t *p_send_dat, uint16_t send_len)
{
  uint8_t sta = 0;

  memcpy (&Ep1Buffer[MAX_PACKET_SIZE], p_send_dat, send_len);

  Ep1DataINFlag = 0;
  R8_UEP1_T_LEN = (uint8_t) send_len;
  PFIC_DisableIRQ (USB_IRQn);
  R8_UEP1_CTRL = R8_UEP1_CTRL & 0xfc; // IN_ACK
  PFIC_EnableIRQ (USB_IRQn);

  return sta;
}


