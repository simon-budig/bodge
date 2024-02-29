/*****************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ****************************************************************************/

#include "CH58x_common.h"

#define dg_log printf

#define THIS_ENDP0_SIZE         64
#define MAX_PACKET_SIZE         64

/* Device Descriptor */
const UINT8 TAB_USB_CDC_DEV_DES[18] =
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
const UINT8 TAB_USB_CDC_CFG_DES[] =
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
const UINT8 My_QueDescr[ ] = { 0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00 };

UINT8 TAB_CDC_LINE_CODING[ ]  =
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
const UINT8 TAB_USB_LID_STR_DES[ ] = { 0x04, 0x03, 0x09, 0x04 };

const UINT8 USB_DEV_PARA_CDC_SERIAL_STR[]      = "WCH121212TS1";
const UINT8 USB_DEV_PARA_CDC_PRODUCT_STR[]     = "USB2.0 To Serial Port";
const UINT8 USB_DEV_PARA_CDC_MANUFACTURE_STR[] = "wch.cn";


typedef struct DevInfo
{
  UINT8 UsbConfig;      // USB configuration flags
  UINT8 UsbAddress;     // USB device address
  UINT8 gSetupReq;      // USB control transfer command code
  UINT8 gSetupLen;      // USB controls the transmission length
  UINT8 gUsbInterCfg;   // USB device interface configuration
  UINT8 gUsbFlag;       // Various operating flags for USB devices,
                        //     bit 0 = bus reset,
                        //     bit 1 = get device descriptor,
                        //     bit 2 = set address,
                        //     bit 3 = get configuration descriptor,
                        //     bit 4 = set configuration
} DevInfo_Parm;

/* Device information */
DevInfo_Parm  devinf;
UINT8 SetupReqCode, SetupLen;

/* Buffers for endpoint hardware and software operations */
__aligned(4) UINT8  Ep0Buffer[MAX_PACKET_SIZE];     // Endpoint 0 Transmit and receive common Endpoint 4 OUT & IN

// The upload address of endpoint 4
__aligned(4) UINT8  Ep1Buffer[MAX_PACKET_SIZE];     // IN
__aligned(4) UINT8  Ep2Buffer[2*MAX_PACKET_SIZE];   // OUT & IN
__aligned(4) UINT8  Ep3Buffer[2*MAX_PACKET_SIZE];   // OUT & IN

// Line Code structure
typedef struct __PACKED _LINE_CODE
{
  UINT32  BaudRate;   /* baud rate */
  UINT8 StopBits;     /* Stop bit counting, 0:1 stop bit, 1:1.5 stop bit, 2:2 stop bit */
  UINT8 ParityType;   /* Check digit, 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space */
  UINT8 DataBits;     /* Data bit count: 5, 6, 7, 8, 16 */
} LINE_CODE, *PLINE_CODE;

/* Data related to the two serial ports */
LINE_CODE Uart0Para;

#define CH341_REG_NUM     10
UINT8 CH341_Reg_Add[CH341_REG_NUM];
UINT8 CH341_Reg_val[CH341_REG_NUM];


/* The serial port parameters are changed in vendor mode */
UINT8 VENSer0ParaChange = 0;

/* In vendor mode, the serial port sends a data flag */
UINT8 VENSer0SendFlag = 0;

/* Modem signal detection in vendor mode */
UINT8 UART0_RTS_Val = 0; // Output indicates that the DTE requests the DCE to send data
UINT8 UART0_DTR_Val = 0; // Output: The data terminal is ready
UINT8 UART0_OUT_Val = 0; // Custom modem signal (CH340 manual)

UINT8 UART0_DCD_Val    = 0;
UINT8 UART0_DCD_Change = 0;

UINT8 UART0_RI_Val    = 0;
UINT8 UART0_RI_Change = 0;

UINT8 UART0_DSR_Val    = 0;
UINT8 UART0_DSR_Change = 0;

UINT8 UART0_CTS_Val    = 0;
UINT8 UART0_CTS_Change = 0;

/* The serial port set by CDC */
UINT8 CDCSetSerIdx      = 0;
UINT8 CDCSer0ParaChange = 0;

typedef struct _USB_SETUP_REQ_ {
    UINT8 bRequestType;
    UINT8 bRequest;
    UINT8 wValueL;
    UINT8 wValueH;
    UINT8 wIndexL;
    UINT8 wIndexH;
    UINT8 wLengthL;
    UINT8 wLengthH;
} USB_SETUP_REQ_t;

#define UsbSetupBuf     ((USB_SETUP_REQ_t *)Ep0Buffer) // USB_SETUP_REQ_t USB_SETUP_REQ_t

/* USB cache definition, all endpoints are defined */
/* Endpoint 1 -- IN status */
UINT8 Ep1DataINFlag = 0;

/* Endpoint 1 transmits data down */
UINT8 Ep1DataOUTFlag = 0;
UINT8 Ep1DataOUTLen  = 0;
__aligned(4) UINT8 Ep1OUTDataBuf[MAX_PACKET_SIZE];

/* Endpoint 2 -- IN Status */
UINT8 Ep2DataINFlag = 0;

/* Endpoint 2 uploads data down */
UINT8 Ep2DataOUTFlag = 0;
UINT8 Ep2DataOUTLen  = 0;
__aligned(4) UINT8 Ep2OUTDataBuf[MAX_PACKET_SIZE];

/* Save the status of USB interrupts - > change the operation mode to several groups */
#define USB_IRQ_FLAG_NUM     4

UINT8 usb_irq_w_idx = 0;
UINT8 usb_irq_r_idx = 0;

volatile UINT8 usb_irq_len[USB_IRQ_FLAG_NUM];
volatile UINT8 usb_irq_pid[USB_IRQ_FLAG_NUM];
volatile UINT8 usb_irq_flag[USB_IRQ_FLAG_NUM];

UINT8 cdc_uart_sta_trans_step = 0;
UINT8 ven_ep1_trans_step      = 0;

/* Endpoint 0 enumerates upload frame processing */
UINT8 ep0_send_buf[256];

/**********************************************************/
UINT8 DevConfig;
UINT16 SetupReqLen;
const UINT8 *pDescr;


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
void USBDevEPnINSetStatus(UINT8 ep_num, UINT8 type, UINT8 sta);

/*******************************************************************************
* Function Name  : CH341RegWrite
* Description    : Write to the register of the CH341
* Input          : reg_add: Write to the register address
*                  reg_val: The value written to the register
* Output         : None
* Return         : None
*******************************************************************************/
void CH341RegWrite(UINT8 reg_add,UINT8 reg_val)
{
  UINT8 find_idx;
  UINT8 find_flag;
  UINT8 i;

  find_flag = 0;
  find_idx = 0;
  for(i=0; i<CH341_REG_NUM; i++)
  {
    if(CH341_Reg_Add[i] == reg_add)
    {
      find_flag = 1;
      break;
    }
    if(CH341_Reg_Add[i] == 0xff)
    {
      find_flag = 0;
      break;
    }
  }
  find_idx = i;
  if(find_flag)
  {
    CH341_Reg_val[find_idx] = reg_val;
  }
  else
  {
    CH341_Reg_Add[find_idx] = reg_add;
    CH341_Reg_val[find_idx] = reg_val;
  }

  switch(reg_add)
  {
    case 0x06:break; // IO
    case 0x07:break; // IO
    case 0x18: // SFR_UART_CTRL --> parameter register of the serial port
    {
      UINT8 reg_uart_ctrl;
      UINT8 data_bit_val;
      UINT8 stop_bit_val;
      UINT8 parity_val;
      UINT8 break_en;

      reg_uart_ctrl = reg_val;
      /* Break bit */
      break_en = (reg_uart_ctrl & 0x40)?(0):(1);
//      SetUART0BreakENStatus(break_en);

      data_bit_val = reg_uart_ctrl & 0x03;
      if      (data_bit_val == 0x00) data_bit_val = HAL_UART_5_BITS_PER_CHAR;
      else if (data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
      else if (data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
      else if (data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;

      stop_bit_val = reg_uart_ctrl & 0x04;
      if (stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
      else              stop_bit_val = HAL_UART_ONE_STOP_BIT;

      parity_val = reg_uart_ctrl & (0x38);
      if      (parity_val == 0x00) parity_val = HAL_UART_NO_PARITY;
      else if (parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
      else if (parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
      else if (parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
      else if (parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;

      // Uart0Para.BaudRate;
      Uart0Para.StopBits   = stop_bit_val;
      Uart0Para.ParityType = parity_val;
      Uart0Para.DataBits   = data_bit_val;

      dg_log("CH341 set para:%d %d %d break:%02x\r\n",data_bit_val,(int)stop_bit_val,parity_val,break_en);

      // Set the registers directly
      VENSer0ParaChange = 1;
      break;
    }
    case 0x25: break;
    case 0x27:
    {
      dg_log("modem set:%02x\r\n",reg_val);
//      SetUART0ModemVendorSta(reg_val);
      break;
    }
  }
}

/*******************************************************************************
* Function Name  : CH341RegRead
* Description    : Reads the registers of the CH341
* Input          : reg_add: The address of the register to be read
*                  reg_val: The value of the read register holds the pointer
* Output         : None
* Return         : Register exists
*******************************************************************************/
UINT8 CH341RegRead(UINT8 reg_add,UINT8 *reg_val)
{
  UINT8 find_flag;
  UINT8 i;

  find_flag = 0;
  *reg_val = 0;
  for(i=0; i<CH341_REG_NUM; i++)
  {
    if(CH341_Reg_Add[i] == reg_add)   // Locate the register with the same address
    {
      find_flag = 1;
      *reg_val = CH341_Reg_val[i];
      break;
    }
    if(CH341_Reg_Add[i] == 0xff)      // Find the first empty one at present
    {
      find_flag = 0;
      *reg_val = 0x00;
      break;
    }
  }

  switch(reg_add)
  {
    case 0x06:
    {
      UINT8  reg_pb_val = 0;
      *reg_val = reg_pb_val;
      break;
    }
    case 0x07:
    {
      UINT8  reg_pc_val = 0;
      *reg_val = reg_pc_val;
      break;
    }
    case 0x18:   // SFR_UART_CTRL --> parameter register of the serial port
    {
      UINT8  reg_uart_ctrl_val;
      UINT8  ram_uart_ctrl_val;

      reg_uart_ctrl_val = R8_UART0_LCR;
      // Reserve the break bit
      ram_uart_ctrl_val = *reg_val;
      reg_uart_ctrl_val |= (ram_uart_ctrl_val & 0x40);
      *reg_val = reg_uart_ctrl_val;

      break;
    }
    case 0x25:  break;
  }

  return find_flag;
}

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
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
  UINT8   i;
  UINT8   j;

  if(R8_USB_INT_FG & RB_UIF_TRANSFER)
  {
    /* Except for the setup package processing */
    if((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN){     // Non-idle
      /* Write directly */
      usb_irq_flag[usb_irq_w_idx] = 1;
      usb_irq_pid[usb_irq_w_idx]  = R8_USB_INT_ST;  // & 0x3f;// (0x30 | 0x0F);
      usb_irq_len[usb_irq_w_idx]  = R8_USB_RX_LEN;

      switch(usb_irq_pid[usb_irq_w_idx]& 0x3f){   // Analyze the current endpoint
        case UIS_TOKEN_OUT | 2:{
          if( R8_USB_INT_FG & RB_U_TOG_OK ){   // Out-of-sync packets are dropped
            R8_UEP2_CTRL ^=  RB_UEP_R_TOG;
            R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xf3) | 0x08; // OUT_NAK
            /* Save the data */
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)
              ((UINT32 *)Ep2OUTDataBuf)[j] = ((UINT32 *)Ep2Buffer)[j];
          }
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 2:{ // endpoint 2# Bulk endpoint upload completed
          R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
          R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xfc) | IN_NAK; // IN_NAK
        }break;
        case UIS_TOKEN_OUT | 1:{
          if( R8_USB_INT_FG & RB_U_TOG_OK ){   // Out-of-sync packets are dropped
            R8_UEP1_CTRL ^=  RB_UEP_R_TOG;
            R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xf3) | 0x08; // OUT_NAK
            /* Save the data */
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)
              ((UINT32 *)Ep1OUTDataBuf)[j] = ((UINT32 *)Ep1Buffer)[j];
          }
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 1:{  // endpoint 1# Batch endpoint upload completed
          R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
          R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xfc) | IN_NAK; // IN_NAK
        }break;
        case UIS_TOKEN_OUT | 0:{    // endpoint 0
          if( R8_USB_INT_FG & RB_U_TOG_OK )   // Out-of-sync packets are dropped
            R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xf3) | 0x08; // OUT_NAK
          else usb_irq_flag[usb_irq_w_idx] = 0;
        }break;
        case UIS_TOKEN_IN | 0:{  // endpoint 0
          R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xfc) | IN_NAK; // IN_NAK
        }break;
      }

      /* Switch to the next write address */
      if(usb_irq_flag[usb_irq_w_idx]){
        usb_irq_w_idx++;
        if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;
      }

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }

    /* setup package processing */
    if(R8_USB_INT_ST & RB_UIS_SETUP_ACT){
      /* Convergence with the previous process - UIS_TOKEN_SETUP */
      /* Write directly */
      usb_irq_flag[usb_irq_w_idx] = 1;
      usb_irq_pid[usb_irq_w_idx]  = UIS_TOKEN_SETUP | 0;  // Keep the previous logo
      usb_irq_len[usb_irq_w_idx]  = 8;
      /* Switch to the next write address */
      usb_irq_w_idx++;
      if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  }
}

UINT8 Ep4DataINFlag;
UINT8 Ep3DataINFlag;

UINT8 Ep1DataOUTLen ;
UINT8 Ep1DataOUTFlag ;

UINT8 Ep3DataOUTFlag = 0;
UINT8 Ep4DataOUTFlag = 0;

/* CH341 related command frames */
#define DEF_VEN_DEBUG_READ    0X95   /* Read two sets of registers */
#define DEF_VEN_DEBUG_WRITE   0X9A   /* Write two sets of registers */
#define DEF_VEN_UART_INIT     0XA1   /* Initialize the serial port */
#define DEF_VEN_UART_M_OUT    0XA4   /* Set the MODEM signal output */
#define DEF_VEN_BUF_CLEAR     0XB2   /* Clear incomplete data */
#define DEF_VEN_I2C_CMD_X     0X54   /* Issue a command for the I2C interface and execute it immediately */
#define DEF_VEN_DELAY_MS      0X5E   /* The specified time is delayed in milliseconds */
#define DEF_VEN_GET_VER       0X5F   /* Get the chip version */

/* Class requests */
//  3.1 Requests---Abstract Control Model
#define DEF_SEND_ENCAPSULATED_COMMAND  0x00
#define DEF_GET_ENCAPSULATED_RESPONSE  0x01
#define DEF_SET_COMM_FEATURE           0x02
#define DEF_GET_COMM_FEATURE           0x03
#define DEF_CLEAR_COMM_FEATURE         0x04
#define DEF_SET_LINE_CODING            0x20   // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING            0x21   // This request allows the host to find out the currently configured line coding.
// #define DEF_SET_CTL_LINE_STE         0X22   // This request generates RS-232/V.24 style control signals.
#define DEF_SET_CONTROL_LINE_STATE     0x22
#define DEF_SEND_BREAK                 0x23

//  3.2 Notifications---Abstract Control Model
#define DEF_NETWORK_CONNECTION         0x00
#define DEF_RESPONSE_AVAILABLE         0x01
#define DEF_SERIAL_STATE               0x20


void USB_IRQProcessHandler( void )   /* USB interrupt service program */
{
  static  PUINT8  pDescr;
  UINT8 len;
  UINT32  bps;
  UINT8   buf[8];
  UINT8   data_dir = 0;   // Data direction
  UINT8   ep_idx, ep_pid;
  UINT8   i;
  UINT8   ep_sta;

  // for(i=0; i<USB_IRQ_FLAG_NUM; i++)
  {
    i = usb_irq_r_idx;

    if(usb_irq_flag[i])
    {
      usb_irq_r_idx++;
      if(usb_irq_r_idx >= USB_IRQ_FLAG_NUM) usb_irq_r_idx = 0;

      switch ( usb_irq_pid[i] & 0x3f )   // Analyze the action token and endpoint number
      {
        case UIS_TOKEN_IN | 4:  // endpoint 4# Batch endpoint upload completed
        {
          Ep4DataINFlag = ~0;
          break;
        }
        case UIS_TOKEN_IN | 3:  // endpoint 3# The bulk endpoint upload is complete
        {
          Ep3DataINFlag = ~0;
          break;
        }
        case UIS_TOKEN_OUT | 2:    // endpoint 2# The batch endpoint upload is complete
        {
          dg_log("usb_rec\n");
          len = usb_irq_len[i];
          {
            // Ep2OUTDataBuf
            for(int i = 0;i<len;i++)
            dg_log("%02x  ",Ep2OUTDataBuf[i]);
            dg_log("\n");

            // Data delivery of CH341
            Ep2DataOUTFlag = 1;
            Ep2DataOUTLen = len;
            VENSer0SendFlag = 1;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP2_CTRL = R8_UEP2_CTRL & 0xf3; // OUT_ACK
            PFIC_EnableIRQ(USB_IRQn);
          }
          break;
        }
        case UIS_TOKEN_IN | 2:  // endpoint 2# Bulk endpoint upload completed
        {
          Ep2DataINFlag = 1;
          break;
        }
        case UIS_TOKEN_OUT | 1:    // endpoint 1# Batch endpoint upload is complete
        {
          dg_log("usb_rec\n");
          len = usb_irq_len[i];
          // Ep1OUTDataBuf
          for(int i = 0;i<len;i++)
          dg_log("%02x  ",Ep1OUTDataBuf[i]);
          dg_log("\n");

          // Data delivery of CH341
          Ep1DataOUTFlag = 1;
          Ep1DataOUTLen = len;
          VENSer0SendFlag = 1;
          PFIC_DisableIRQ(USB_IRQn);
          R8_UEP1_CTRL = R8_UEP1_CTRL & 0xf3; // OUT_ACK
          PFIC_EnableIRQ(USB_IRQn);
          break;
        }
        case UIS_TOKEN_IN | 1:   // endpoint 1# Interrupt the endpoint upload is complete
        {
          Ep1DataINFlag = 1;
          break;
        }
        case UIS_TOKEN_SETUP | 0:    // endpoint 0# SETUP
        {
          len = usb_irq_len[i];
          if(len == sizeof(USB_SETUP_REQ))
          {
            SetupLen = UsbSetupBuf->wLengthL;
            if(UsbSetupBuf->wLengthH) SetupLen = 0xff;

            len = 0;                                                 // The default is Succeeded and the upload is 0 length
            SetupReqCode = UsbSetupBuf->bRequest;

            /* Data direction */
            data_dir = USB_REQ_TYP_OUT;
            if(UsbSetupBuf->bRequestType & USB_REQ_TYP_IN) data_dir = USB_REQ_TYP_IN;

            /* Vendor request */
            if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_VENDOR )
            {
              switch(SetupReqCode)
              {
                case DEF_VEN_DEBUG_WRITE:  // Write two sets of 0X9A
                {
                  UINT32 bps = 0;
                  UINT8 write_reg_add1;
                  UINT8 write_reg_add2;
                  UINT8 write_reg_val1;
                  UINT8 write_reg_val2;

                  len = 0;

                  write_reg_add1 = Ep0Buffer[2];
                  write_reg_add2 = Ep0Buffer[3];
                  write_reg_val1 = Ep0Buffer[4];
                  write_reg_val2 = Ep0Buffer[5];

                  /* This group is the register that sets the baud rate */
                  if((write_reg_add1 == 0x12)&&(write_reg_add2 == 0x13))
                  {
                    /* Baud rate processing uses calculated values */
                    if((UsbSetupBuf->wIndexL==0x87)&&(UsbSetupBuf->wIndexH==0xf3))
                    {
                      bps = 921600;  // 13 * 921600 = 11980800
                    }
                    else if((UsbSetupBuf->wIndexL==0x87)&&(UsbSetupBuf->wIndexH==0xd9))
                    {
                      bps = 307200;  // 39 * 307200 = 11980800
                    }

                    // System Frequency: 36923077
                    else if( UsbSetupBuf->wIndexL == 0x88 )
                    {
                      UINT32 CalClock;
                      UINT8 CalDiv;

                      CalClock = 36923077 / 8;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    else if( UsbSetupBuf->wIndexL == 0x89 )
                    {
                      UINT32 CalClock;
                      UINT8 CalDiv;

                      CalClock = 36923077 / 8 / 256;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    // System frequency: 32000000
                    else if( UsbSetupBuf->wIndexL == 0x8A )
                    {
                      UINT32 CalClock;
                      UINT8 CalDiv;

                      CalClock = 32000000 / 8;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    else if( UsbSetupBuf->wIndexL == 0x8B )
                    {
                      UINT32 CalClock;
                      UINT8 CalDiv;

                      CalClock = 32000000 / 8 / 256;
                      CalDiv = 0 - UsbSetupBuf->wIndexH;
                      bps = CalClock / CalDiv;
                    }
                    else  // 340
                    {
                      UINT32 CalClock;
                      UINT8 CalDiv;

                      // 115384
                      if((UsbSetupBuf->wIndexL & 0x7f) == 3)
                      {
                        CalClock = 6000000;
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 2)
                      {
                        CalClock = 750000;  // 6000000 / 8
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 1)
                      {
                        CalClock = 93750; // Divide by 64
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else if((UsbSetupBuf->wIndexL & 0x7f) == 0)
                      {
                        CalClock = 11719;  // About 512
                        CalDiv = 0 - UsbSetupBuf->wIndexH;
                        bps = CalClock / CalDiv;
                      }
                      else
                      {
                        bps = 115200;
                      }
                    }
                    Uart0Para.BaudRate = bps;
                    dg_log("CH341 set bps:%d\r\n",(int)bps);
                    // UART0BpsSet(bps);
                  }
                  else
                  {
                    CH341RegWrite(write_reg_add1,write_reg_val1);
                    CH341RegWrite(write_reg_add2,write_reg_val2);
                  }

                  break;
                }
                case DEF_VEN_DEBUG_READ:   // Require callback data 0X95 /* Read two sets of registers */
                {
                  UINT8 read_reg_add1;
                  UINT8 read_reg_add2;
                  UINT8 read_reg_val1;
                  UINT8 read_reg_val2;

                  read_reg_add1 = UsbSetupBuf->wValueL;
                  read_reg_add2 = UsbSetupBuf->wValueH;

                  CH341RegRead(read_reg_add1,&read_reg_val1);
                  CH341RegRead(read_reg_add2,&read_reg_val2);

                  len = 2;
                  pDescr = buf;
                  buf[0] = read_reg_val1;
                  buf[1] = read_reg_val2;
                  SetupLen = len;
                  memcpy(Ep0Buffer, pDescr, len);

                  break;
                }
                // The A1 command also needs to initialize the serial port
                case DEF_VEN_UART_INIT:  // Initialize the serial port 0XA1
                {
                  UINT8 reg_uart_ctrl;
                  UINT8  parity_val;
                  UINT8  data_bit_val;
                  UINT8  stop_bit_val;
                  UINT8  uart_reg1_val;
                  UINT8  uart_reg2_val;
                  UINT8  uart_set_m;

                  len = 0;

                  if(Ep0Buffer[2] & 0x80)
                  {
                    reg_uart_ctrl = Ep0Buffer[3];

                    data_bit_val = reg_uart_ctrl & 0x03;
                    if      (data_bit_val == 0x00) data_bit_val = HAL_UART_5_BITS_PER_CHAR;
                    else if (data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
                    else if (data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
                    else if (data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;

                    stop_bit_val = reg_uart_ctrl & 0x04;
                    if (stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
                    else stop_bit_val = HAL_UART_ONE_STOP_BIT;

                    parity_val = reg_uart_ctrl & (0x38);
                    if      (parity_val == 0x00) parity_val = HAL_UART_NO_PARITY;
                    else if (parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
                    else if (parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
                    else if (parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
                    else if (parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;

                    // Uart0Para.BaudRate;
                    Uart0Para.StopBits = stop_bit_val;
                    Uart0Para.ParityType = parity_val;
                    Uart0Para.DataBits = data_bit_val;

                    // Set the registers directly
                   // UART0ParaSet(data_bit_val, stop_bit_val,parity_val);

                    uart_set_m = 0;
                    uart_reg1_val = UsbSetupBuf->wIndexL;
                    uart_reg2_val = UsbSetupBuf->wIndexH;

                    if(uart_reg1_val & (1<<6))  // Judgment No. 6
                    {
                      uart_set_m = 1;
                    }
                    else
                    {
                      uart_set_m = 1;
                      uart_reg1_val = uart_reg1_val & 0xC7;
                    }

                    if(uart_set_m)
                    {
                      /* Baud rate processing uses calculated values */
                      if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xf3))
                      {
                        bps = 921600;  // 13 * 921600 = 11980800
                      }
                      else if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xd9))
                      {
                        bps = 307200;  // 39 * 307200 = 11980800
                      }

                      // System Frequency: 36923077
                      else if( uart_reg1_val == 0xC8 )
                      {
                        UINT32 CalClock;
                        UINT8 CalDiv;

                        CalClock = 36923077 / 8;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      else if( uart_reg1_val == 0xC9 )
                      {
                        UINT32 CalClock;
                        UINT8 CalDiv;

                        CalClock = 36923077 / 8 / 256;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      // System frequency: 32000000
                      else if( uart_reg1_val == 0xCA )
                      {
                        UINT32 CalClock;
                        UINT8 CalDiv;

                        CalClock = 32000000 / 8;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      else if( uart_reg1_val == 0xCB )
                      {
                        UINT32 CalClock;
                        UINT8 CalDiv;

                        CalClock = 32000000 / 8 / 256;
                        CalDiv = 0 - uart_reg2_val;
                        bps = CalClock / CalDiv;
                      }
                      else  // 340
                      {
                        UINT32 CalClock;
                        UINT8 CalDiv;

                        // 115384
                        if((uart_reg1_val & 0x7f) == 3)
                        {
                          CalClock = 6000000;
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 2)
                        {
                          CalClock = 750000;  // 6000000 / 8
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 1)
                        {
                          CalClock = 93750; // Divide by 64
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else if((uart_reg1_val & 0x7f) == 0)
                        {
                          CalClock = 11719;  // About 512
                          CalDiv = 0 - uart_reg2_val;
                          bps = CalClock / CalDiv;
                        }
                        else
                        {
                          bps = 115200;
                        }
                      }
                      Uart0Para.BaudRate = bps;
                      dg_log("CH341 set bps:%d\r\n",(int)bps);
                      // UART0BpsSet(bps);
                    }
                  }
                  break;
                }
                case DEF_VEN_UART_M_OUT:  // Set the MODEM signal output 0XA4
                {
                  UINT8 reg_pb_out;

                  len = 0;
                  reg_pb_out = Ep0Buffer[2];
                  if(reg_pb_out & (1<<4)) UART0_OUT_Val = 1;
                  else UART0_OUT_Val = 0;
                  break;
                }
                case DEF_VEN_BUF_CLEAR: // 0XB2  /* Clear incomplete data */
                {
                  len = 0;

                  // Can be reinitialized
                  VENSer0ParaChange = 1; // Reinitialize to erase all data
                  break;
                }
                case DEF_VEN_I2C_CMD_X:  // 0X54 Issue the command for the I2C interface and execute it immediately
                {
                  len = 0;
                  break;
                }
                case DEF_VEN_DELAY_MS:  // 0X5E Specify the time in milliseconds
                {
                  len = 0;
                  break;
                }
                case DEF_VEN_GET_VER:   // 0X5E Get the chip version // Need to return data --> version number
                {
                  len = 2;
                  pDescr = buf;
                  buf[0] = 0x30;
                  buf[1] = 0x00;
                  SetupLen = len;
                  memcpy( Ep0Buffer, pDescr, len );

                  break;
                }
                default:
                  // len = 0xFF;
                  len = 0;
                  break;
              }
            }
            // Standard Request
            else if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
            {
              switch( SetupReqCode )  // Request code
              {
                case USB_GET_DESCRIPTOR:  // Get a descriptor
                {
                  switch( UsbSetupBuf->wValueH )
                  {
                    case 1: // Device descriptor
                    {
                      memcpy(ep0_send_buf,
                             &TAB_USB_CDC_DEV_DES[0],
                             sizeof( TAB_USB_CDC_DEV_DES ));

                      pDescr = ep0_send_buf;
                      len = sizeof( TAB_USB_CDC_DEV_DES );
                      break;
                    }
                    case 2:  // Configure descriptors
                    {
                      memcpy(ep0_send_buf,
                             &TAB_USB_CDC_CFG_DES[0],
                             sizeof( TAB_USB_CDC_CFG_DES ));
                      pDescr = ep0_send_buf;
                      len = sizeof( TAB_USB_CDC_CFG_DES );
                      break;
                    }
                    case 3:  // String descriptors
                    {
                      dg_log("str %d\r\n",UsbSetupBuf->wValueL);
                      dg_log("str %d\r\n",UsbSetupBuf->wValueL);
                      switch(UsbSetupBuf->wValueL)
                      {
                        case 0:  // Language descriptors
                        {
                          pDescr = (PUINT8)( &TAB_USB_LID_STR_DES[0] );
                          len = sizeof( TAB_USB_LID_STR_DES );

                          break;
                        }
                        case 1:   // iManufacturer
                        case 2:   // iProduct
                        case 3:   // iSerialNumber
                        {
                          UINT8 ep0_str_len;
                          UINT8 *p_send;
                          UINT8 *manu_str;
                          UINT8 tmp;

                          /* Take the length */
                          if(UsbSetupBuf->wValueL == 1)
                            manu_str = (UINT8 *)USB_DEV_PARA_CDC_MANUFACTURE_STR;
                          else if(UsbSetupBuf->wValueL == 2)
                            manu_str = (UINT8 *)USB_DEV_PARA_CDC_PRODUCT_STR;
                          else if(UsbSetupBuf->wValueL == 3)
                            manu_str = (UINT8 *)USB_DEV_PARA_CDC_SERIAL_STR;
                          ep0_str_len = (UINT8)strlen((char *)manu_str);
                          p_send = ep0_send_buf;
                          *p_send++ = ep0_str_len*2 + 2;
                          *p_send++ = 0x03;
                          for(tmp = 0; tmp<ep0_str_len; tmp++)
                          {
                            *p_send++ = manu_str[tmp];
                            *p_send++ = 0x00;
                          }

                          pDescr = ep0_send_buf;
                          len = ep0_send_buf[0];

                          break;
                        }
                        default:
                          len = 0xFF;    // Unsupported descriptor types
                          break;
                      }
                      break;
                    }
                    case 6:  // Device Qualified Descriptor
                    {
                      pDescr = (PUINT8)( &My_QueDescr[0] );
                      len = sizeof( My_QueDescr );
                      break;
                    }
                    default:
                      len = 0xFF;                                  // Unsupported descriptor types
                      break;
                  }
                  if ( SetupLen > len ) SetupLen = len;            // Limit the overall length
                  len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // The length of this transfer
                  memcpy( Ep0Buffer, pDescr, len );                 /* Load the feed */
                  SetupLen -= len;
                  pDescr += len;

                  break;
                }
                case USB_SET_ADDRESS:  // Set the address
                {
                  dg_log("SET_ADDRESS:%d\r\n",UsbSetupBuf->wValueL);
                  devinf.gUsbFlag |= DEF_BIT_USB_ADDRESS;
                  devinf.UsbAddress = UsbSetupBuf->wValueL;    // Staging USB device addresses

                  break;
                }
                case USB_GET_CONFIGURATION:
                {
                  dg_log("GET_CONFIGURATION\r\n");
                  Ep0Buffer[0] = devinf.UsbConfig;
                  if ( SetupLen >= 1 ) len = 1;

                  break;
                }
                case USB_SET_CONFIGURATION:
                {
                  dg_log("SET_CONFIGURATION\r\n");
                  devinf.gUsbFlag |= DEF_BIT_USB_SET_CFG;
                  devinf.UsbConfig = UsbSetupBuf->wValueL;
                  break;
                }
                case USB_CLEAR_FEATURE:
                {
                  dg_log("CLEAR_FEATURE\r\n");
                  len = 0;
                  /* Clear the device */
                  if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                  {
                    R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
                    R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;

                    // The state variable is reset
                    Ep1DataINFlag = 1;
                    Ep2DataINFlag = 1;
                    Ep3DataINFlag = 1;
                    Ep4DataINFlag = 1;

                    Ep1DataOUTFlag = 0;
                    Ep2DataOUTFlag = 0;
                    Ep3DataOUTFlag = 0;
                    Ep4DataOUTFlag = 0;

                    cdc_uart_sta_trans_step = 0;
                    ven_ep1_trans_step = 0;
                  }
                  else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )  // endpoint
                  {
                    switch( UsbSetupBuf->wIndexL )   // Judgment endpoint
                    {
                      case 0x84: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x04: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      case 0x83: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x03: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      case 0x82: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x02: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      case 0x81: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
                      case 0x01: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
                      default: len = 0xFF;  break;
                    }
                  }
                  else len = 0xFF;                                  // It's not that the endpoint isn't supported

                  break;
                }
                case USB_GET_INTERFACE:
                {
                  dg_log("GET_INTERFACE\r\n");
                  Ep0Buffer[0] = 0x00;
                  if ( SetupLen >= 1 ) len = 1;
                  break;
                }
                case USB_GET_STATUS:
                {
                  dg_log("GET_STATUS\r\n");
                  Ep0Buffer[0] = 0x00;
                  Ep0Buffer[1] = 0x00;
                  if ( SetupLen >= 2 ) len = 2;
                  else len = SetupLen;
                  break;
                }
                default:
                  len = 0xFF;                                       // The operation failed
                  break;
              }
            }
            /* Class requests */
            else if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS )
            {
              /* The host is downloaded */
              if(data_dir == USB_REQ_TYP_OUT)
              {
                switch( SetupReqCode )  // Request code
                {
                  case DEF_SET_LINE_CODING: /* SET_LINE_CODING */
                  {
                    UINT8 i;
                    dg_log("SET_LINE_CODING\r\n");
                    for(i=0; i<8; i++)
                    {
                      dg_log("%02x ",Ep0Buffer[i]);
                    }
                    dg_log("\r\n");
                    if( Ep0Buffer[ 4 ] == 0x00 )
                    {
                      CDCSetSerIdx = 0;
                      len = 0x00;
                    }
                    else if( Ep0Buffer[ 4 ] == 0x02 )
                    {
                      CDCSetSerIdx = 1;
                      len = 0x00;
                    }
                    else len = 0xFF;
                    break;
                  }
                  case DEF_SET_CONTROL_LINE_STATE:  /* SET_CONTROL_LINE_STATE */
                  {
                    UINT8  carrier_sta;
                    UINT8  present_sta;
                    /* Line status */
                    dg_log("ctl %02x %02x\r\n",Ep0Buffer[2],Ep0Buffer[3]);
                    carrier_sta = Ep0Buffer[2] & (1<<1);   // RTS status
                    present_sta = Ep0Buffer[2] & (1<<0);   // DTR status
                    len = 0;
                    break;
                  }
                  default:
                  {
                    dg_log("CDC ReqCode%x\r\n",SetupReqCode);
                    len = 0xFF;                                       // The operation failed
                    break;
                  }
                }
              }
              /* Device upload */
              else
              {
                switch( SetupReqCode )  // Request code
                {
                  case DEF_GET_LINE_CODING: /* GET_LINE_CODING */
                  {
                    dg_log("GET_LINE_CODING:%d\r\n",Ep0Buffer[ 4 ]);
                    pDescr = Ep0Buffer;
                    len = sizeof( LINE_CODE );
                    ( ( PLINE_CODE )Ep0Buffer )->BaudRate   = Uart0Para.BaudRate;
                    ( ( PLINE_CODE )Ep0Buffer )->StopBits   = Uart0Para.StopBits;
                    ( ( PLINE_CODE )Ep0Buffer )->ParityType = Uart0Para.ParityType;
                    ( ( PLINE_CODE )Ep0Buffer )->DataBits   = Uart0Para.DataBits;
                    break;
                  }
                  case DEF_SERIAL_STATE:
                  {
                    dg_log("GET_SERIAL_STATE:%d\r\n",Ep0Buffer[ 4 ]);
                    // SetupLen determines the overall length
                    len = 2;
                    CDCSetSerIdx = 0;
                    Ep0Buffer[0] = 0;
                    Ep0Buffer[1] = 0;
                    break;
                  }
                  default:
                  {
                    dg_log("CDC ReqCode%x\r\n",SetupReqCode);
                    len = 0xFF;                                       // The operation failed
                    break;
                  }
                }
              }
            }

            else len = 0xFF;   /* fail */
          }
          else
          {
            len = 0xFF; // The length of the SETUP package is incorrect
          }
          if ( len == 0xFF )  // The operation failed
          {
            SetupReqCode = 0xFF;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  // STALL
            PFIC_EnableIRQ(USB_IRQn);
          }
          else if ( len <= THIS_ENDP0_SIZE )  // A packet of 0 length is returned in the upload or status phase
          {
            if( SetupReqCode ==  USB_SET_ADDRESS)  // Set the address 0x05
            {
//              dg_log("add in:%d\r\n",len);
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_SET_CONFIGURATION)  // Set the configuration value 0x09
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_GET_DESCRIPTOR)  // Get the descriptor 0x06
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_UART_INIT )  // 0XA1 Initialize the serial port
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_DEBUG_WRITE )  // 0X9A
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_VEN_UART_M_OUT )  // 0XA4
            {
              R8_UEP0_T_LEN = len;
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  DEF_SET_CONTROL_LINE_STATE )  // 0x22
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else if( SetupReqCode ==  USB_CLEAR_FEATURE )  // 0x01
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
              PFIC_EnableIRQ(USB_IRQn);
            }
            else
            {
              if(data_dir == USB_REQ_TYP_IN)   // Upload is currently required
              {
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_T_LEN = len;
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;  // The default packet is DATA1
                PFIC_EnableIRQ(USB_IRQn);
              }
              else                            // Downloading is currently required
              {
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_T_LEN = len;
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;  // The default packet is DATA1
                PFIC_EnableIRQ(USB_IRQn);
              }
            }
          }
          else  // Downloading data or others
          {
            // Although it has not yet reached the state stage, it is necessary to preset and upload 0-length packets in advance to prevent the host from entering the state phase in advance
            R8_UEP0_T_LEN = 0;
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;  // The default packet is DATA1
            PFIC_EnableIRQ(USB_IRQn);
          }
          break;
        }
        case UIS_TOKEN_IN | 0:      // endpoint 0# IN  UIS_TOKEN_IN
        {
          switch( SetupReqCode )
          {
            /* Simple handling of SETUP commands */
            case USB_GET_DESCRIPTOR:  // 0x06 Get a descriptor
            {
              len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;  // The length of this transfer
              memcpy( Ep0Buffer, pDescr, len );                    /* Load the feed */
              SetupLen -= len;
              pDescr += len;

              if(len)
              {
                R8_UEP0_T_LEN = len;
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_CTRL ^=  RB_UEP_T_TOG;
                USBDevEPnINSetStatus(ENDP0, ENDP_TYPE_IN, IN_ACK);
                PFIC_EnableIRQ(USB_IRQn);
              }
              else
              {
                R8_UEP0_T_LEN = len;
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
                PFIC_EnableIRQ(USB_IRQn);
              }
              break;
            }
            case USB_SET_ADDRESS:   // 0x05
            {
              R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | (devinf.UsbAddress);
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);
//              dg_log("add in deal\r\n");

              break;
            }
            // Maker reads
            case DEF_VEN_DEBUG_READ:     // 0X95
            case DEF_VEN_GET_VER:        // 0X5F
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);

              break;
            }
            case DEF_GET_LINE_CODING:  // 0x21
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);

              break;
            }
            case DEF_SET_LINE_CODING:   // 0x20
            {
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);
              break;
            }
            default:
            {
              R8_UEP0_T_LEN = 0; // The state phase completes an interrupt or forces the upload of 0-length packets to end the control transmission
              PFIC_DisableIRQ(USB_IRQn);
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              PFIC_EnableIRQ(USB_IRQn);

              break;
            }
          }
          break;
        }
        case UIS_TOKEN_OUT | 0:  // endpoint 0# OUT
        {
          len = usb_irq_len[i];
          if(len)
          {
            switch(SetupReqCode)
            {
              /* Set the serial port */
              case DEF_SET_LINE_CODING:
              {
                UINT32 set_bps;
                UINT8  data_bit;
                UINT8  stop_bit;
                UINT8  ver_bit;
                UINT8  set_stop_bit;

                memcpy(&set_bps,Ep0Buffer,4);
                stop_bit = Ep0Buffer[4];
                ver_bit = Ep0Buffer[5];
                data_bit = Ep0Buffer[6];

                dg_log("LINE_CODING %d %d %d %d %d\r\n",CDCSetSerIdx
                                     ,(int)set_bps
                                     ,data_bit
                                     ,stop_bit
                                     ,ver_bit);

                  Uart0Para.BaudRate = set_bps;
                  Uart0Para.StopBits = stop_bit;
                  Uart0Para.ParityType = ver_bit;
                  Uart0Para.DataBits = data_bit;
                  CDCSer0ParaChange = 1;

                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
                PFIC_EnableIRQ(USB_IRQn);
                break;
              }
              default:
                PFIC_DisableIRQ(USB_IRQn);
                R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
                PFIC_EnableIRQ(USB_IRQn);
                break;
            }
          }
          else
          {
            PFIC_DisableIRQ(USB_IRQn);
            R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_NAK;
            PFIC_EnableIRQ(USB_IRQn);
          }
          break;
        }
        default:
          ep_idx = 0xff;
          break;
      }

      PFIC_DisableIRQ(USB_IRQn);
      usb_irq_flag[i] = 0;
      PFIC_EnableIRQ(USB_IRQn);
    }
  }

  if ( R8_USB_INT_FG & RB_UIF_BUS_RST )  // USB bus reset
  {
      R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
      R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
      R8_UEP3_CTRL = UEP_T_RES_NAK;
      R8_UEP4_CTRL = UEP_T_RES_NAK;

    cdc_uart_sta_trans_step = 0;
    ven_ep1_trans_step = 0;

    R8_USB_DEV_AD = 0x00;
    devinf.UsbAddress = 0;

    R8_USB_INT_FG = RB_UIF_BUS_RST;             // Clear the break sign
  }
  else if (  R8_USB_INT_FG & RB_UIF_SUSPEND )  // USB bus hang/wake complete
  {
    if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )    // Suspend
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
    else                                     // awaken
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

    cdc_uart_sta_trans_step = 0;
    ven_ep1_trans_step = 0;

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
void USBDevEPnINSetStatus(UINT8 ep_num, UINT8 type, UINT8 sta)
{
  UINT8 *p_UEPn_CTRL;

  p_UEPn_CTRL = (UINT8 *)(USB_BASE_ADDR + 0x22 + ep_num * 4);
  if(type == ENDP_TYPE_IN) *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03))) | sta;
  else *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03<<2))) | (sta<<2);
}

/*******************************************************************************
* Function Name  : USBParaInit
* Description    : USB parameter initialization, cache and flags
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBParaInit(void)
{
  Ep1DataINFlag = 1;
  Ep1DataOUTFlag = 0;
  Ep2DataINFlag = 1;
  Ep2DataOUTFlag = 0;
  Ep3DataINFlag = 1;
  Ep3DataOUTFlag = 0;
  Ep4DataINFlag = 1;
  Ep4DataOUTFlag = 0;
}


/*******************************************************************************
* Function Name  : InitCDCDevice
* Description    : Initialize the CDC device
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitCDCDevice(void)
{
  /* Initialize the cache */
  USBParaInit();

  R8_USB_CTRL = 0x00;                                                 // Set the mode first

//  1. Endpoint Assignment:
//  Endpoint 0
//  Endpoint 1: IN and OUT (Data Interface)
//  Endpoint 2: IN and OUT (Data Interface)
//  Endpoint 3: IN (interface 23 combination, interrupt upload)
//  Endpoint 4: IN (interface 01 combination, interrupt upload)

  R8_UEP4_1_MOD = RB_UEP4_TX_EN|RB_UEP1_TX_EN|RB_UEP1_RX_EN;

  /* Single 64-Byte Receive Buffer (OUT), Single 64-Byte Send Buffer (IN) */
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_TX_EN;

  R16_UEP0_DMA = (UINT16)(UINT32)&Ep0Buffer[0];
  R16_UEP1_DMA = (UINT16)(UINT32)&Ep1Buffer[0];
  R16_UEP2_DMA = (UINT16)(UINT32)&Ep2Buffer[0];
  R16_UEP3_DMA = (UINT16)(UINT32)&Ep3Buffer[0];
  // R16_UEP4_DMA = (UINT16)(UINT32)&Ep2Buffer[0];

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

  // Boot the USB device and DMA and automatically return the NAK until the interrupt flag is cleared during the outage
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

  // Clear the break sign
  R8_USB_INT_FG = 0xFF;

  // Unified program query?
  // Turn on interrupt, pend, transfer complete, bus reset
  // R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
  R8_USB_INT_EN = RB_UIE_TRANSFER ;
  PFIC_EnableIRQ(USB_IRQn);

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
void InitUSBDefPara(void)
{
  UINT8 i;

  Uart0Para.BaudRate = 115200;
  Uart0Para.DataBits = HAL_UART_8_BITS_PER_CHAR;
  Uart0Para.ParityType = HAL_UART_NO_PARITY;
  Uart0Para.StopBits = HAL_UART_ONE_STOP_BIT;

  VENSer0ParaChange = 0;
  VENSer0SendFlag = 0;
  CDCSetSerIdx = 0;
  CDCSer0ParaChange = 0;

  for(i=0; i<CH341_REG_NUM; i++)
  {
    CH341_Reg_Add[i] = 0xff;
    CH341_Reg_val[i] = 0x00;
  }

  UART0_DCD_Val = 0;
  UART0_RI_Val = 0;
  UART0_DSR_Val = 0;
  UART0_CTS_Val = 0;

  UART0_RTS_Val = 0; // Output indicates that the DTE requests the DCE to send data
  UART0_DTR_Val = 0; // Output: The data terminal is ready
  UART0_OUT_Val = 0; // Custom modem signal (CH340 manual)

  for(i=0; i<USB_IRQ_FLAG_NUM; i++)
  {
    usb_irq_flag[i] = 0;
  }
}

/*******************************************************************************
* Function Name  : InitUSBDevice
* Description    : Initialize USB
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitUSBDevice(void)
{
  InitCDCDevice();
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
UINT8 SendUSBData(UINT8 *p_send_dat,UINT16 send_len)
{
  UINT8 sta = 0;

  memcpy(&Ep1Buffer[MAX_PACKET_SIZE],p_send_dat,send_len);

  Ep1DataINFlag = 0;
  R8_UEP1_T_LEN = (UINT8)send_len;
  PFIC_DisableIRQ(USB_IRQn);
  R8_UEP1_CTRL = R8_UEP1_CTRL & 0xfc; // IN_ACK
  PFIC_EnableIRQ(USB_IRQn);

  return sta;
}


