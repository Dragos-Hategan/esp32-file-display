#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#define UI_COLOR_BG_DARK             lv_color_hex(0x101214)
#define UI_COLOR_CARD_DARK           lv_color_hex(0x202327)
#define UI_COLOR_BORDER_DARK         lv_color_hex(0x2D3034)
#define UI_COLOR_BUTTON_BORDER_DARK  lv_color_hex(0xBBAAFF)
#define UI_COLOR_INDICATOR_OFF_DARK  lv_color_hex(0x00055F)
#define UI_COLOR_TEXT_DARK           lv_color_hex(0xDCDCDC)
#define UI_COLOR_ACCENT_DARK         lv_color_hex(0x7D5FFF)

#define UI_COLOR_BG_LIGHT             lv_color_hex(0xF2F2F7)
#define UI_COLOR_CARD_LIGHT           lv_color_hex(0xFFFFFF)
#define UI_COLOR_BORDER_LIGHT         lv_color_hex(0xC6C6C8)
#define UI_COLOR_BUTTON_BORDER_LIGHT  lv_color_hex(0x30D158)   
#define UI_COLOR_INDICATOR_OFF_LIGHT  lv_color_hex(0xA3A7AD)
#define UI_COLOR_TEXT_LIGHT           lv_color_hex(0x1C1C1E)
#define UI_COLOR_ACCENT_LIGHT         lv_color_hex(0x34C759)   

void styles_init_colors(void);
void styles_set_bg_color(lv_obj_t *obj);

void styles_build_screen(lv_obj_t *screen);
void styles_build_button(lv_obj_t *button);
void styles_build_switch(lv_obj_t *switch_button);
void styles_build_textarea(lv_obj_t *textarea);
void styles_build_dropdown(lv_obj_t *dropdown);
void styles_build_msgbox(lv_obj_t *mbox);
void styles_build_keyboard(lv_obj_t *kbd);


#ifdef __cplusplus
}
#endif