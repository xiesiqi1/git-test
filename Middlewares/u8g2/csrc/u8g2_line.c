/*

  u8g2_line.c

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/

#include "u8g2.h"

void u8g2_DrawLine(u8g2_t *u8g2, u8g2_uint_t x0, u8g2_uint_t y0, u8g2_uint_t x1, u8g2_uint_t y1)
{
  u8g2_int_t dx, dy;
  u8g2_int_t err;
  u8g2_int_t y_step;
  u8g2_int_t x0i = (u8g2_int_t)x0;
  u8g2_int_t y0i = (u8g2_int_t)y0;
  u8g2_int_t x1i = (u8g2_int_t)x1;
  u8g2_int_t y1i = (u8g2_int_t)y1;
  /* 2026-09-28: 这里原本还有一行 "u8g2_int_t tmp;", 但全函数都没用到它,
   * Keil 会报 warning #177-D。删掉以保持工程 0 warning。 */

  dx = x1i - x0i;
  dy = y1i - y0i;

  if ( dy == 0 )
  {
    if ( x0i <= x1i )
      u8g2_DrawHLine(u8g2, x0, y0, x1 - x0 + 1);
    else
      u8g2_DrawHLine(u8g2, x1, y0, x0 - x1 + 1);
    return;
  }

  if ( dx == 0 )
  {
    if ( y0i <= y1i )
      u8g2_DrawVLine(u8g2, x0, y0, y1 - y0 + 1);
    else
      u8g2_DrawVLine(u8g2, x0, y1, y0 - y1 + 1);
    return;
  }

  if ( dx < 0 )
  {
    dx = -dx;
    x0i = x1i;
    y0i = y1i;
    dy = -dy;
  }

  if ( dy < 0 )
  {
    dy = -dy;
    err = dx / 2;
    y_step = -1;
  }
  else
  {
    err = -dx / 2;
    y_step = 1;
  }

  for(;;)
  {
    u8g2_DrawPixel(u8g2, (u8g2_uint_t)x0i, (u8g2_uint_t)y0i);
    if ( x0i == x1i )
      break;
    err += dy;
    if ( err > 0 )
    {
      err -= dx;
      y0i += y_step;
    }
    x0i++;
  }
}
