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
void styles_set_bg_color(lv_obj_t *obj, lv_style_selector_t selector);
void styles_set_text_color(lv_obj_t *obj, lv_style_selector_t selector);
void styles_set_border_color(lv_obj_t *obj, lv_style_selector_t selector);

void styles_set_card_color(lv_obj_t *obj, lv_style_selector_t selector);

void styles_set_screen(lv_obj_t *screen);
void styles_set_button(lv_obj_t *button);
void styles_set_list_button(lv_obj_t *list_button);
void styles_set_switch(lv_obj_t *switch_button);
void styles_set_slider(lv_obj_t *slider);
void styles_set_textarea(lv_obj_t *textarea);
void styles_set_dropdown(lv_obj_t *dropdown);
void styles_set_msgbox(lv_obj_t *mbox);
void styles_set_dialog(lv_obj_t *dialog);
void styles_set_keyboard(lv_obj_t *kbd);


#ifdef __cplusplus
}
#endif