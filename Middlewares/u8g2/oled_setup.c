#include "u8g2.h"

/* Buffer for SH1106 128x64 full frame: 128*64/8 = 1024 bytes */
static uint8_t u8g2_frame_buf[1024];

uint8_t *u8g2_m_16_8_f(uint8_t *page_cnt)
{
  *page_cnt = 8;
  return u8g2_frame_buf;
}

/* Setup function for SH1106 128x64 winstar full buffer mode */
void u8g2_Setup_sh1106_128x64_winstar_f(u8g2_t *u8g2, const u8g2_cb_t *rotation,
    u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;
  u8g2_SetupDisplay(u8g2, u8x8_d_sh1106_128x64_winstar, u8x8_cad_001, byte_cb, gpio_and_delay_cb);
  buf = u8g2_m_16_8_f(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}

/* ---------------------------------------------------------------------------
 *  下面这些是"替身函数(stub)":
 *  原生裁剪版 u8g2hal 为了省 Flash 删掉了几个 .c 文件(u8g2_font.c /
 *  u8x8_fonts.c / u8x8_debounce.c ...), 但剩下的代码里还有对它们的引用,
 *  不补上就会 "Undefined symbol"。本工程并不用到这些功能, 所以给个空实现即可。
 *
 *  ★ 2026-09-28 注意: 本工程【补回了 u8g2_font.c 和 u8x8_8x8.c】
 *    (因为 OLED 要显示文字)。于是有三个替身函数和真身撞车了 —— 已删除:
 *        u8g2_SetFontPosBaseline   -> 真身在 csrc/u8g2_font.c
 *        u8x8_DrawUTF8             -> 真身在 csrc/u8x8_8x8.c
 *        u8x8_GetUTF8Len           -> 真身在 csrc/u8x8_8x8.c
 *    如果以后又把这些文件从 Keil 工程里删掉, 记得把这三个替身加回来,
 *    否则链接会报未定义。
 *    (u8x8_GetMenuEvent 是例外: 它的真身在 u8x8_debounce.c, 本工程没要那个
 *     文件, 所以这个替身【必须保留】。)
 * ------------------------------------------------------------------------- */

uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8) { (void)u8x8; return 0; }

uint8_t u8x8_capture_get_pixel_1(uint16_t x, uint16_t y, uint8_t *dest_ptr, uint8_t tile_width)
{ (void)x; (void)y; (void)dest_ptr; (void)tile_width; return 0; }
uint8_t u8x8_capture_get_pixel_2(uint16_t x, uint16_t y, uint8_t *dest_ptr, uint8_t tile_width)
{ (void)x; (void)y; (void)dest_ptr; (void)tile_width; return 0; }
void u8x8_capture_write_pbm_pre(uint8_t tile_width, uint8_t tile_height, void (*out)(const char *s))
{ (void)tile_width; (void)tile_height; (void)out; }
void u8x8_capture_write_pbm_buffer(uint8_t *buffer, uint8_t tile_width, uint8_t tile_height,
    uint8_t (*get_pixel)(uint16_t x, uint16_t y, uint8_t *dest_ptr, uint8_t tile_width),
    void (*out)(const char *s))
{ (void)buffer; (void)tile_width; (void)tile_height; (void)get_pixel; (void)out; }
void u8x8_capture_write_xbm_pre(uint8_t tile_width, uint8_t tile_height, void (*out)(const char *s))
{ (void)tile_width; (void)tile_height; (void)out; }
void u8x8_capture_write_xbm_buffer(uint8_t *buffer, uint8_t tile_width, uint8_t tile_height,
    uint8_t (*get_pixel)(uint16_t x, uint16_t y, uint8_t *dest_ptr, uint8_t tile_width),
    void (*out)(const char *s))
{ (void)buffer; (void)tile_width; (void)tile_height; (void)get_pixel; (void)out; }
