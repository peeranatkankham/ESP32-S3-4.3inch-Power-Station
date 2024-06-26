// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_add_flag(ui_Screen1, LV_OBJ_FLAG_CHECKABLE);     /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel5 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel5, 400);
    lv_obj_set_height(ui_Panel5, 350);
    lv_obj_set_x(ui_Panel5, -180);
    lv_obj_set_y(ui_Panel5, -51);
    lv_obj_set_align(ui_Panel5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel5, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0x101418), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel5, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_battery_indicator = lv_arc_create(ui_Screen1);
    lv_obj_set_width(ui_battery_indicator, 350);
    lv_obj_set_height(ui_battery_indicator, 350);
    lv_obj_set_x(ui_battery_indicator, -180);
    lv_obj_set_y(ui_battery_indicator, -32);
    lv_obj_set_align(ui_battery_indicator, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_battery_indicator,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE |
                      LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                      LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_arc_set_value(ui_battery_indicator, 50);
    lv_obj_set_style_arc_width(ui_battery_indicator, 35, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_battery_indicator, lv_color_hex(0x1FD2FC), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_battery_indicator, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_battery_indicator, 35, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_battery_indicator, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_battery_indicator, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_battery_level = lv_label_create(ui_battery_indicator);
    lv_obj_set_width(ui_battery_level, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_battery_level, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_battery_level, LV_ALIGN_CENTER);
    lv_label_set_text(ui_battery_level, "00");
    lv_obj_set_style_text_align(ui_battery_level, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_battery_level, &ui_font_digital160, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel1 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel1, 320);
    lv_obj_set_height(ui_Panel1, 135);
    lv_obj_set_x(ui_Panel1, 220);
    lv_obj_set_y(ui_Panel1, -160);
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel1, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x8B44FF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container1 = lv_obj_create(ui_Panel1);
    lv_obj_remove_style_all(ui_Container1);
    lv_obj_set_width(ui_Container1, 250);
    lv_obj_set_height(ui_Container1, 120);
    lv_obj_set_x(ui_Container1, -20);
    lv_obj_set_y(ui_Container1, 0);
    lv_obj_set_align(ui_Container1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Power = lv_label_create(ui_Container1);
    lv_obj_set_width(ui_Power, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Power, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Power, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Power, "0");
    lv_obj_set_style_text_font(ui_Power, &ui_font_digital140, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container2 = lv_obj_create(ui_Panel1);
    lv_obj_remove_style_all(ui_Container2);
    lv_obj_set_width(ui_Container2, 50);
    lv_obj_set_height(ui_Container2, 135);
    lv_obj_set_x(ui_Container2, 130);
    lv_obj_set_y(ui_Container2, 0);
    lv_obj_set_align(ui_Container2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container2, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_watt = lv_label_create(ui_Container2);
    lv_obj_set_width(ui_watt, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_watt, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_watt, 0);
    lv_obj_set_y(ui_watt, 30);
    lv_obj_set_align(ui_watt, LV_ALIGN_CENTER);
    lv_label_set_text(ui_watt, "W");
    lv_obj_set_style_text_font(ui_watt, &ui_font_digital_60, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image1 = lv_img_create(ui_Container2);
    lv_img_set_src(ui_Image1, &ui_img_power_png);
    lv_obj_set_width(ui_Image1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image1, 0);
    lv_obj_set_y(ui_Image1, -30);
    lv_obj_set_align(ui_Image1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel2 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel2, 150);
    lv_obj_set_height(ui_Panel2, 150);
    lv_obj_set_x(ui_Panel2, 137);
    lv_obj_set_y(ui_Panel2, 0);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel2, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0x3FFE68), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Volt = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Volt, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Volt, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Volt, 0);
    lv_obj_set_y(ui_Volt, -27);
    lv_obj_set_align(ui_Volt, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Volt, "0");
    lv_obj_set_style_text_font(ui_Volt, &ui_font_digital_80, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_volttext = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_volttext, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_volttext, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_volttext, 0);
    lv_obj_set_y(ui_volttext, 48);
    lv_obj_set_align(ui_volttext, LV_ALIGN_CENTER);
    lv_label_set_text(ui_volttext, "Volt");
    lv_obj_set_style_text_font(ui_volttext, &ui_font_digital50, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel7 = lv_obj_create(ui_Panel2);
    lv_obj_set_width(ui_Panel7, 110);
    lv_obj_set_height(ui_Panel7, 5);
    lv_obj_set_x(ui_Panel7, 0);
    lv_obj_set_y(ui_Panel7, 18);
    lv_obj_set_align(ui_Panel7, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel7, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel7, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel7, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel3 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel3, 320);
    lv_obj_set_height(ui_Panel3, 135);
    lv_obj_set_x(ui_Panel3, 220);
    lv_obj_set_y(ui_Panel3, 160);
    lv_obj_set_align(ui_Panel3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel3, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel3, lv_color_hex(0xACFB39), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Time = lv_label_create(ui_Panel3);
    lv_obj_set_width(ui_Time, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Time, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Time, -40);
    lv_obj_set_y(ui_Time, 0);
    lv_obj_set_align(ui_Time, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Time, "0");
    lv_obj_set_style_text_font(ui_Time, &ui_font_digital140, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Time1 = lv_label_create(ui_Panel3);
    lv_obj_set_width(ui_Time1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Time1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Time1, 0);
    lv_obj_set_y(ui_Time1, 30);
    lv_obj_set_align(ui_Time1, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(ui_Time1, "min");
    lv_obj_set_style_text_font(ui_Time1, &ui_font_digital_60, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image2 = lv_img_create(ui_Panel3);
    lv_img_set_src(ui_Image2, &ui_img_clock_png);
    lv_obj_set_width(ui_Image2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Image2, 0);
    lv_obj_set_y(ui_Image2, -30);
    lv_obj_set_align(ui_Image2, LV_ALIGN_RIGHT_MID);
    lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel9 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel9, 400);
    lv_obj_set_height(ui_Panel9, 80);
    lv_obj_set_x(ui_Panel9, -180);
    lv_obj_set_y(ui_Panel9, 188);
    lv_obj_set_align(ui_Panel9, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel9, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel9, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel9, lv_color_hex(0x101418), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel9, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel9, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button1 = lv_btn_create(ui_Panel9);
    lv_obj_set_width(ui_Button1, 150);
    lv_obj_set_height(ui_Button1, 80);
    lv_obj_set_x(ui_Button1, -20);
    lv_obj_set_y(ui_Button1, 0);
    lv_obj_set_align(ui_Button1, LV_ALIGN_LEFT_MID);
    lv_obj_add_flag(ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button1, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0xBB1AF4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image5 = lv_img_create(ui_Button1);
    lv_img_set_src(ui_Image5, &ui_img_533029405);
    lv_obj_set_width(ui_Image5, LV_SIZE_CONTENT);   /// 64
    lv_obj_set_height(ui_Image5, LV_SIZE_CONTENT);    /// 64
    lv_obj_set_align(ui_Image5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Temp = lv_label_create(ui_Panel9);
    lv_obj_set_width(ui_Temp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Temp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Temp, -55);
    lv_obj_set_y(ui_Temp, 0);
    lv_obj_set_align(ui_Temp, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(ui_Temp, "0");
    lv_obj_set_style_text_font(ui_Temp, &ui_font_digital50, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image3 = lv_img_create(ui_Panel9);
    lv_img_set_src(ui_Image3, &ui_img_celsius_png);
    lv_obj_set_width(ui_Image3, LV_SIZE_CONTENT);   /// 50
    lv_obj_set_height(ui_Image3, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_align(ui_Image3, LV_ALIGN_RIGHT_MID);
    lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel4 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel4, 150);
    lv_obj_set_height(ui_Panel4, 150);
    lv_obj_set_x(ui_Panel4, 303);
    lv_obj_set_y(ui_Panel4, 0);
    lv_obj_set_align(ui_Panel4, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel4, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel4, lv_color_hex(0x2EDEE9), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Amp = lv_label_create(ui_Panel4);
    lv_obj_set_width(ui_Amp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Amp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Amp, 0);
    lv_obj_set_y(ui_Amp, -27);
    lv_obj_set_align(ui_Amp, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Amp, "0");
    lv_obj_set_style_text_font(ui_Amp, &ui_font_digital_80, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_amptext = lv_label_create(ui_Panel4);
    lv_obj_set_width(ui_amptext, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_amptext, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_amptext, 0);
    lv_obj_set_y(ui_amptext, 48);
    lv_obj_set_align(ui_amptext, LV_ALIGN_CENTER);
    lv_label_set_text(ui_amptext, "AMP");
    lv_obj_set_style_text_font(ui_amptext, &ui_font_digital50, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel8 = lv_obj_create(ui_Panel4);
    lv_obj_set_width(ui_Panel8, 110);
    lv_obj_set_height(ui_Panel8, 5);
    lv_obj_set_x(ui_Panel8, 0);
    lv_obj_set_y(ui_Panel8, 18);
    lv_obj_set_align(ui_Panel8, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel8, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel8, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Panel8, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel8, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_ALL, NULL);

}
