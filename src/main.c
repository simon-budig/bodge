/*******************************************************************************
 * Copyright (c) 2024 Simon Budig
 ******************************************************************************/

#include "CH58x_common.h"

#include "gfxfont.h"
#include "font.h"

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


void
set_pixel (uint8_t *fb, int x, int y)
{
  if (x < 0 || x >= 44 || y < 0 || y >= 11)
    return;

  fb[(x/8)*11 + y] |= (1 << (x%8));
}

void
render_text (const char *text,
             int         x0,
             int         y0,
             uint8_t    *fb)
{
  int i, j;
  uint8_t val;
  const GFXfont *font = &bbtempo8pt7b;
  const GFXglyph *glyph;

  memset (fb, 0, 66);

  if (x0 < 0)
    {
      x0 = -x0;
      for (i = 0; text[i] != '\0'; i++)
        {
          if (text[i] < font->first || text[i] > font->last)
            continue;
          glyph = &font->glyph[text[i] - font->first];
          x0 -= glyph->xAdvance;
        }
    }

  for (i = 0; text[i] != '\0'; i++)
    {
      const uint8_t *bitdata;

      if (text[i] < font->first || text[i] > font->last)
        continue;

      glyph = &font->glyph[text[i] - font->first];
      bitdata = font->bitmap + glyph->bitmapOffset;
      for (j = 0; j < glyph->width * glyph->height; j++)
        {
          val = (bitdata[j/8] >> (7-(j%8))) & 0x01;
          if (!val)
            continue;
          set_pixel (fb,
                     x0 + glyph->xOffset + j % glyph->width,
                     y0 + glyph->yOffset + j / glyph->width);
        }

      x0 += glyph->xAdvance;
    }
}

int
main ()
{
  int i;
  uint8_t *indata;
  uint8_t indata_len = 0;
  char text[] = "                      ";
  int db = 0;

  SetSysClock (CLK_SOURCE_PLL_60MHz);

  board_pin_init ();

  TMR0_TimerInit (1500);
  TMR0_ITCfg (ENABLE, TMR0_3_IT_CYC_END);
  PFIC_EnableIRQ (TMR0_IRQn);

  InitUSBDefPara ();
  InitUSBDevice ();
  PFIC_EnableIRQ (USB_IRQn);

  next_fb = fb[db];

  while (1)
    {
      indata_len = 0;
      USB_IRQProcessHandler (&indata, &indata_len);

      for (i = 0; i < indata_len; i++)
        {
          memmove (text, text + 1, sizeof (text) - 1);
          text[sizeof (text) - 2] = indata[i];
        }

      if (indata_len > 0)
        {
          db = 1 - db;
          render_text (text, -44, 8, fb[db]);
          next_fb = fb[db];
          SendUSBData (indata, indata_len);
        }
    }
}

