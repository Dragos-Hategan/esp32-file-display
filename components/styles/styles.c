#include "styles.h"
#include "settings.h"

typedef struct{
    lv_color_t color_bg;
    lv_color_t color_card;
    lv_color_t color_border;
    lv_color_t color_button_border;
    lv_color_t color_indicator_off;
    lv_color_t color_text;
    lv_color_t color_accent;
}style_colors_t;

static style_colors_t dark_colors;
static style_colors_t light_colors;

static style_colors_t get_theme(bool dark);

static inline lv_style_selector_t style_sel(lv_part_t part, lv_state_t state)
{
    return (lv_style_selector_t)((lv_style_selector_t)part | (lv_style_selector_t)state);
}

void styles_init_colors(void)
{
    dark_colors.color_bg = UI_COLOR_BG_DARK;
    dark_colors.color_card = UI_COLOR_CARD_DARK;
    dark_colors.color_border = UI_COLOR_BORDER_DARK;
    dark_colors.color_button_border = UI_COLOR_BUTTON_BORDER_DARK;
    dark_colors.color_indicator_off = UI_COLOR_INDICATOR_OFF_DARK;
    dark_colors.color_text = UI_COLOR_TEXT_DARK;
    dark_colors.color_accent = UI_COLOR_ACCENT_DARK;

    light_colors.color_bg = UI_COLOR_BG_LIGHT;
    light_colors.color_card = UI_COLOR_CARD_LIGHT;
    light_colors.color_border = UI_COLOR_BORDER_LIGHT;
    light_colors.color_button_border = UI_COLOR_BUTTON_BORDER_LIGHT;
    light_colors.color_indicator_off = UI_COLOR_INDICATOR_OFF_LIGHT;
    light_colors.color_text = UI_COLOR_TEXT_LIGHT;
    light_colors.color_accent = UI_COLOR_ACCENT_LIGHT;
}

void styles_set_bg_color(lv_obj_t *obj, lv_style_selector_t selector)
{
    if (!obj) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(obj, colors.color_bg, selector);
}

void styles_set_border_color(lv_obj_t *obj, lv_style_selector_t selector)
{
    if (!obj) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_border_color(obj, colors.color_border, selector);
}


void styles_set_text_color(lv_obj_t *obj, lv_style_selector_t selector)
{   
    if (!obj) {
        return;
    }    
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_text_color(obj, colors.color_text, selector);    
}

void styles_set_card_color(lv_obj_t *obj, lv_style_selector_t selector)
{
    if (!obj) {
        return;
    }   
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(obj, colors.color_card, 0);
}

void styles_set_screen(lv_obj_t *screen)
{
    if (!screen) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(screen, colors.color_bg, 0);
    lv_obj_set_style_text_color(screen, colors.color_text, 0);
}

void styles_set_button(lv_obj_t *button)
{
    if (!button) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(button, colors.color_accent, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(button, colors.color_button_border, LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
    lv_obj_set_style_text_color(button, colors.color_text, LV_PART_MAIN);
}

void styles_set_list_button(lv_obj_t *list_button)
{
    if (!list_button) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(list_button, colors.color_card, LV_PART_MAIN);
    lv_obj_set_style_border_color(list_button, colors.color_border, LV_PART_MAIN);
    lv_obj_set_style_text_color(list_button, colors.color_text, LV_PART_MAIN);
    lv_obj_set_style_text_color(list_button, colors.color_text, LV_PART_ITEMS);
}

void styles_set_dropdown(lv_obj_t *dropdown)
{
    if (!dropdown) {
        return;
    }    
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    if (dropdown) {
        lv_obj_set_style_bg_color(dropdown, colors.color_card, LV_PART_MAIN);
        lv_obj_set_style_bg_color(dropdown, colors.color_border, LV_PART_SCROLLBAR);

        lv_obj_set_style_bg_opa(dropdown, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_text_color(dropdown, colors.color_text, 0);

        /* Subtle selection: keep base bg, just a 1px accent border on selected item */
        lv_obj_set_style_bg_opa(dropdown, LV_OPA_TRANSP, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED));
        lv_obj_set_style_bg_opa(dropdown, LV_OPA_TRANSP, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED | LV_STATE_PRESSED));
        lv_obj_set_style_border_color(dropdown, colors.color_button_border, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED));
        lv_obj_set_style_border_color(dropdown, colors.color_button_border, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED | LV_STATE_PRESSED));
        lv_obj_set_style_border_width(dropdown, 1, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED));
        lv_obj_set_style_border_width(dropdown, 1, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED | LV_STATE_PRESSED));
        lv_obj_set_style_text_color(dropdown, colors.color_text, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED));
        lv_obj_set_style_text_color(dropdown, colors.color_text, style_sel(LV_PART_SELECTED, LV_STATE_CHECKED | LV_STATE_PRESSED));

        lv_obj_set_style_border_color(dropdown, colors.color_border, LV_PART_SCROLLBAR);
        lv_obj_set_style_border_color(dropdown, colors.color_border, LV_PART_MAIN);

        lv_obj_set_style_border_width(dropdown, 1, LV_PART_MAIN);
    }
}

void styles_set_switch(lv_obj_t *switch_button)
{
    if (!switch_button) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(switch_button, colors.color_card, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(switch_button, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(switch_button, colors.color_accent, LV_PART_KNOB);
    lv_obj_set_style_border_color(switch_button, colors.color_button_border, LV_PART_KNOB);
    lv_obj_set_style_bg_color(switch_button, colors.color_indicator_off, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(switch_button, 2, LV_PART_KNOB);  
}

void styles_set_slider(lv_obj_t *slider)
{
    if (!slider) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(slider, colors.color_border, 0);
    lv_obj_set_style_bg_color(slider, colors.color_accent, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, colors.color_accent, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, colors.color_button_border, LV_PART_KNOB);
}

void styles_set_textarea(lv_obj_t *textarea)
{
    if (!textarea) {
        return;
    }    
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());

    lv_obj_set_style_bg_color(textarea, lv_color_lighten(colors.color_card, 50), 0);
    lv_obj_set_style_bg_opa(textarea, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(textarea, colors.color_border, 0);
    lv_obj_set_style_border_width(textarea, 1, 0);
    lv_obj_set_style_text_color(textarea, colors.color_text, 0);
}

void styles_set_msgbox(lv_obj_t *mbox)
{
    if (!mbox) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());    

    lv_obj_set_style_bg_color(mbox, colors.color_card, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(mbox, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(mbox, colors.color_border, LV_PART_MAIN);
    lv_obj_set_style_border_width(mbox, 1, LV_PART_MAIN);
    lv_obj_set_style_text_color(mbox, colors.color_text, LV_PART_MAIN);
    lv_obj_set_style_text_color(mbox, colors.color_text, LV_PART_ITEMS);
}

void styles_set_dialog(lv_obj_t *dialog)
{
    if (!dialog) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag()); 

    lv_obj_set_style_border_color(dialog, colors.color_border, 0);
    lv_obj_set_style_bg_color(dialog, colors.color_card, 0);
    lv_obj_set_style_text_color(dialog, colors.color_text, 0);
}

void styles_set_keyboard(lv_obj_t *kbd)
{
    if (!kbd) {
        return;
    }
    style_colors_t colors = get_theme(settings_get_dark_theme_flag());
    
    lv_obj_set_style_bg_color(kbd, colors.color_card, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(kbd, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(kbd, colors.color_border, LV_PART_MAIN);
    lv_obj_set_style_border_width(kbd, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(kbd, 6, LV_PART_MAIN);
    lv_obj_set_style_text_color(kbd, colors.color_text, LV_PART_MAIN);

    /* Keys */
    lv_obj_set_style_bg_color(kbd, colors.color_card, LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(kbd, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_border_color(kbd, colors.color_border, LV_PART_ITEMS);
    lv_obj_set_style_border_width(kbd, 1, LV_PART_ITEMS);
    lv_obj_set_style_text_color(kbd, colors.color_text, LV_PART_ITEMS);

    /* Pressed/checked/focused keys: subtle dark instead of accent */
    lv_obj_set_style_bg_color(kbd, colors.color_border, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(kbd, colors.color_border, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(kbd, colors.color_border, LV_PART_ITEMS | LV_STATE_FOCUSED);
    lv_obj_set_style_bg_opa(kbd, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(kbd, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(kbd, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_FOCUSED);
    lv_obj_set_style_text_color(kbd, colors.color_text, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(kbd, colors.color_text, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(kbd, colors.color_text, LV_PART_ITEMS | LV_STATE_FOCUSED);
}


static style_colors_t get_theme(bool dark)
{   
    if (dark){
        return dark_colors;
    }

    return light_colors;
}