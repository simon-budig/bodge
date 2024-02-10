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

void board_led_init(void)
{
    GPIOB_SetBits(GPIO_Pin_4);
    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);
}

void board_led_toggle(void)
{
    GPIOB_InverseBits(GPIO_Pin_4);
}

void board_led_set(uint8_t set)
{
    if (set)
        GPIOB_ResetBits(GPIO_Pin_4);
    else
        GPIOB_SetBits(GPIO_Pin_4);
}

int main()
{
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    board_led_init();

    while(1)
    {
      board_led_set(0); 
      mDelaymS(1);
      board_led_set(1); 
      mDelaymS(199);
    }
}

