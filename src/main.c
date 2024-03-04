/*******************************************************************************
 * Copyright (c) 2024 Simon Budig
 ******************************************************************************/

#include "CH58x_common.h"

#define PA(x) (0x80 | x)
#define PB(x) (0x00 | x)

#define NUM_PINS 23

static const uint8_t matrixpins[NUM_PINS] = {
  PA(15), PB(18), PB(0),  PB(7),
  PA(12), PA(10), PA(11), PB(9),
  PB(8),  PB(15), PB(14), PB(13),
  PB(12), PB(5),  PA(4),  PB(3),
  PB(4),  PB(2),  PB(1),  PB(23),
  PB(21), PB(20), PB(19),
};

static int matrix_mask_a = 0;
static int matrix_mask_b = 0;

static uint8_t fb[2][(((NUM_PINS-1) + 7)/8) * (NUM_PINS-1)] = {
  {  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, },
  {
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, },
};
static volatile uint8_t *next_fb = NULL;
static volatile uint8_t *cur_fb = NULL;

static volatile int ticks = 0;

__INTERRUPT
__HIGH_CODE
void
TMR0_IRQHandler (void)
{
  static int cur_pos = 0;
  static int count = 0;
  uint8_t x, y;
  uint8_t val;

  ticks ++;

  if (TMR0_GetITFlag (TMR0_3_IT_CYC_END))
    {
      TMR0_ClearITFlag (TMR0_3_IT_CYC_END);

      GPIOA_ResetBits (matrix_mask_a);
      GPIOA_ModeCfg (matrix_mask_a, GPIO_ModeIN_Floating);
      GPIOB_ResetBits (matrix_mask_b);
      GPIOB_ModeCfg (matrix_mask_b, GPIO_ModeIN_Floating);

      cur_pos = (cur_pos + 1) % ((NUM_PINS-1)*(NUM_PINS-1));
      if (!cur_pos)
        {
          if (next_fb)
            cur_fb = next_fb;
          next_fb = NULL;
          count++;
        }

      if (!cur_fb)
        return;

      x = cur_pos / (NUM_PINS-1);
      y = cur_pos % (NUM_PINS-1);

      /* funky framebuffer mapping */
      val = (cur_fb[y/2 + (x/4)*11] >> (y%2 + (x%4)*2)) & 0x01;

      /* fix up swapped first two pixels */
      if (cur_pos <= 1)
        y = 1-y;

      /* fix up for charlieplexing (y must be != x) */
      if (y >= x)
        y++;

      if (val)
        {
          int xbits = (1 << (matrixpins[x] & 0x7f));
          int ybits = (1 << (matrixpins[y] & 0x7f));

          if (matrixpins[x] & 0x80)
            {
              GPIOA_SetBits (xbits);
              GPIOA_ModeCfg (xbits, GPIO_ModeOut_PP_20mA);
            }
          else
            {
              GPIOB_SetBits (xbits);
              GPIOB_ModeCfg (xbits, GPIO_ModeOut_PP_20mA);
            }

          if (matrixpins[y] & 0x80)
            {
              GPIOA_ResetBits (ybits);
              GPIOA_ModeCfg (ybits, GPIO_ModeOut_PP_20mA);
            }
          else
            {
              GPIOB_ResetBits (ybits);
              GPIOB_ModeCfg (ybits, GPIO_ModeOut_PP_20mA);
            }
        }
    }
}


void
board_pin_init (void)
{
  int i;

  for (i = 0; i < NUM_PINS; i++)
    {
      if (matrixpins[i] & 0x80)
        matrix_mask_a |= (1 << (matrixpins[i] & 0x7f));
      else
        matrix_mask_b |= (1 << (matrixpins[i] & 0x7f));
    }

  GPIOA_ResetBits (matrix_mask_a);
  GPIOA_ModeCfg (matrix_mask_a, GPIO_ModeIN_Floating);
  GPIOB_ResetBits (matrix_mask_b);
  GPIOB_ModeCfg (matrix_mask_b, GPIO_ModeIN_Floating);
}


int
main ()
{
  int i;
  uint8_t *indata;
  uint8_t indata_len = 0;

  SetSysClock (CLK_SOURCE_PLL_60MHz);

  board_pin_init ();

  TMR0_TimerInit (1500);
  TMR0_ITCfg (ENABLE, TMR0_3_IT_CYC_END);
  PFIC_EnableIRQ (TMR0_IRQn);

  InitUSBDefPara ();
  InitUSBDevice ();
  PFIC_EnableIRQ (USB_IRQn);

  next_fb = fb[0];

  while (1)
    {
      indata_len = 0;
      USB_IRQProcessHandler (&indata, &indata_len);

      for (i = 0; i < indata_len; i++)
        indata[i] ^= 0x20;

      if (indata_len > 0)
        SendUSBData (indata, indata_len);

    //if ((ticks % 10000) == 0)
    //  SendUSBData ("a\r\n", 3);
    }
}

