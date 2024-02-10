/********************************* (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2022/01/25
 * Description        :
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 ******************************************************************************/

#include "CH58x_common.h"

#define PA(x) (0x80 | x)
#define PB(x) (0x00 | x)

#define NUM_PINS 23

static const uint8_t matrixpins[NUM_PINS] = {
  PA(15),
  PB(18),
  PB(0),
  PB(7),
  PA(12),
  PA(10),
  PA(11),
  PB(9),
  PB(8),
  PB(15),
  PB(14),
  PB(13),
  PB(12),
  PB(5),
  PA(4),
  PB(3),
  PB(4),
  PB(2),
  PB(1),
  PB(23),
  PB(21),
  PB(20),
  PB(19),
};

static int matrix_mask_a = 0;
static int matrix_mask_b = 0;


__INTERRUPT
__HIGH_CODE
void
TMR0_IRQHandler (void)
{
  static int cur_pos = 0;
  static int count = 0;
  uint8_t x, y;
  int r;

  if (TMR0_GetITFlag (TMR0_3_IT_CYC_END))
    {
      TMR0_ClearITFlag (TMR0_3_IT_CYC_END);

      GPIOA_ResetBits (matrix_mask_a);
      GPIOA_ModeCfg (matrix_mask_a, GPIO_ModeIN_Floating);
      GPIOB_ResetBits (matrix_mask_b);
      GPIOB_ModeCfg (matrix_mask_b, GPIO_ModeIN_Floating);

      cur_pos = (cur_pos + 1) % 484;
      if (!cur_pos)
        count++;

      x = cur_pos / (NUM_PINS-1);
      y = cur_pos % (NUM_PINS-1);

      r = ((3*x+y + count/3) / 8) % 2;

      if (y >= x)
        y++;


      if (r)
        {
          if (matrixpins[x] & 0x80)
            {
              GPIOA_SetBits (1 << (matrixpins[x] & 0x7f));
              GPIOA_ModeCfg (1 << (matrixpins[x] & 0x7f), GPIO_ModeOut_PP_5mA);
            }
          else
            {
              GPIOB_SetBits (1 << (matrixpins[x] & 0x7f));
              GPIOB_ModeCfg (1 << (matrixpins[x] & 0x7f), GPIO_ModeOut_PP_5mA);
            }

          if (matrixpins[y] & 0x80)
            {
              GPIOA_ResetBits (1 << (matrixpins[y] & 0x7f));
              GPIOA_ModeCfg (1 << (matrixpins[y] & 0x7f), GPIO_ModeOut_PP_5mA);
            }
          else
            {
              GPIOB_ResetBits (1 << (matrixpins[y] & 0x7f));
              GPIOB_ModeCfg (1 << (matrixpins[y] & 0x7f), GPIO_ModeOut_PP_5mA);
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
  SetSysClock (CLK_SOURCE_PLL_60MHz);

  board_pin_init ();

  TMR0_TimerInit (2000);
  TMR0_ITCfg (ENABLE, TMR0_3_IT_CYC_END);
  PFIC_EnableIRQ (TMR0_IRQn);

  while (1)
    {
      mDelaymS (200);
    }
}

