#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#define UI_COLOR_BG_DARK             lv_color_hex(0x101214)
#define UI_COLOR_CARD_DARK           lv_color_hex(0x202327)
#define UI_COLOR_BORDER_DARK         lv_color_hex(0x2D3034)
#define UI_COLOR_BUTTON_BORDER_DARK  lv_color_hex(0xBBAAFF)
#define UI_COLOR_INDICATOR_OFF_DARK  lv_color_hex(0xA3A7AD)
#define UI_COLOR_TEXT_DARK           lv_color_hex(0xDCDCDC)
#define UI_COLOR_ACCENT_DARK         lv_color_hex(0x7D5FFF)

#define UI_COLOR_BG_LIGHT             lv_color_hex(0xF2F2F7)
#define UI_COLOR_CARD_LIGHT           lv_color_hex(0xFFFFFF)
#define UI_COLOR_BORDER_LIGHT         lv_color_hex(0xC6C6C8)
#define UI_COLOR_BUTTON_BORDER_LIGHT  lv_color_hex(0x2EBCBD)   
#define UI_COLOR_INDICATOR_OFF_LIGHT  lv_color_hex(0xA3A7AD)
#define UI_COLOR_TEXT_LIGHT           lv_color_hex(0x1C1C1E)
#define UI_COLOR_ACCENT_LIGHT         lv_color_hex(0x34C759)   

/**
 * @brief Initialize the color palette for light and dark themes.
 *
 * Loads the UI_COLOR_* macro values into internal theme structures.
 * Call once before applying any styles to LVGL objects.
 */
void styles_init_colors(void);

/**
 * @brief Set the background color based on the current theme.
 * @param obj LVGL object to be styled (ignored if NULL).
 * @param selector Style selector (part/state) that receives the color.
 */
void styles_set_bg_color(lv_obj_t *obj, lv_style_selector_t selector);

/**
 * @brief Set the text color based on the current theme.
 * @param obj LVGL object to be styled (ignored if NULL).
 * @param selector Style selector (part/state) that receives the color.
 */
void styles_set_text_color(lv_obj_t *obj, lv_style_selector_t selector);

/**
 * @brief Set the border color based on the current theme.
 * @param obj LVGL object to be styled (ignored if NULL).
 * @param selector Style selector (part/state) that receives the color.
 */
void styles_set_border_color(lv_obj_t *obj, lv_style_selector_t selector);

/**
 * @brief Style a card/container background with the current theme color.
 * @param obj LVGL object to be styled (ignored if NULL).
 * @param selector Kept for API symmetry; color is applied on the main part (0).
 */
void styles_set_card_color(lv_obj_t *obj, lv_style_selector_t selector);

/**
 * @brief Apply the theme color scheme to an LVGL screen.
 * @param screen Screen object (lv_scr_act or similar); ignored if NULL.
 */
void styles_set_screen(lv_obj_t *screen);

/**
 * @brief Style a button using the accent color of the current theme.
 * @param button Button object; ignored if NULL.
 */
void styles_set_button(lv_obj_t *button);

/**
 * @brief Style LV_LIST items/buttons according to the current theme.
 * @param list_button List item/button object; ignored if NULL.
 */
void styles_set_list_button(lv_obj_t *list_button);

/**
 * @brief Style an LVGL switch according to the current theme.
 * @param switch_button Switch object; ignored if NULL.
 */
void styles_set_switch(lv_obj_t *switch_button);

/**
 * @brief Style a slider according to the current theme.
 * @param slider Slider object; ignored if NULL.
 */
void styles_set_slider(lv_obj_t *slider);

/**
 * @brief Style an lv_arc according to the current theme.
 * @param arc Arc object; ignored if NULL.
 */
void styles_set_arc(lv_obj_t *arc);

/**
 * @brief Style a textarea according to the current theme.
 * @param textarea Textarea object; ignored if NULL.
 */
void styles_set_textarea(lv_obj_t *textarea);

/**
 * @brief Style a dropdown, including its selected state, using the current theme.
 * @param dropdown Dropdown object; ignored if NULL.
 */
void styles_set_dropdown(lv_obj_t *dropdown);

/**
 * @brief Style a message box according to the current theme.
 * @param mbox Message box object; ignored if NULL.
 */
void styles_set_msgbox(lv_obj_t *mbox);

/**
 * @brief Style a dialog (generic container) according to the current theme.
 * @param dialog Dialog object; ignored if NULL.
 */
void styles_set_dialog(lv_obj_t *dialog);

/**
 * @brief Style the LVGL keyboard, including key states, using the current theme.
 * @param kbd Keyboard object; ignored if NULL.
 */
void styles_set_keyboard(lv_obj_t *kbd);


#ifdef __cplusplus
}
#endif
