
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include "ui.h"
#define TFT_BL 2
#define GFX_BL DF_GFX_BL  // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
  40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
  45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
  5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
  8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);

//  ST7262 IPS LCD 800x480
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
  bus,
  800 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
  480 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
  1 /* pclk_active_neg */, 14000000 /* prefer_speed */, true /* auto_flush */);

#include "touch.h"

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (touch_has_signal()) {
    if (touch_touched()) {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    } else if (touch_released()) {
      data->state = LV_INDEV_STATE_REL;
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

#define RX_PIN 17
#define TX_PIN 18

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  // while (!Serial);
  // Init Display
  gfx->begin();
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  lv_init();
  delay(5);
  touch_init();
  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef ESP32
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4);
#endif
  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 4);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();
    lv_obj_add_event_cb(ui_USBC, ui_event_USBC, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ACOutput, ui_event_ACOutput, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_DCOutput, ui_event_DCOutput, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_USBA, ui_event_USBA, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Fan, ui_event_Fan, LV_EVENT_ALL, NULL);
  }
}

bool toggleState = false;
bool toggleStateACOutput = false;
bool toggleStateDCOutput = false;
bool toggleStateUSBA = false;
bool toggleStateFan = false;


void ui_event_USBC(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED) {
    _ui_state_modify(ui_Switch2, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
    _ui_checked_set_text_value(ui_Label3, target, "ON", "OFF");
    toggleusbc(e);
     unsigned long timeout = millis() + 4000;  // Adjust as needed


    while (millis() < timeout) {
      if (toggleState) {
        Serial2.write('i');
      } else {
        Serial2.write('j');
      }
      delay(10);
      // Read serial response
      if (Serial2.available() > 0) {
        String response = Serial2.readStringUntil('\n');
        if (response.equals("turnoffc") && toggleState) {
          break;  // Break the loop if "turnoff" received and toggleState is true
        } else if (response.equals("turnonc") && !toggleState) {
          break;  // Break the loop if "turnon" received and toggleState is false
        }
      }
      delay(200);
    }

    toggleState = !toggleState;
  }
}
void ui_event_ACOutput(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED) {
    _ui_state_modify(ui_Switch3, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
    _ui_checked_set_text_value(ui_Label5, target, "ON", "OFF");
    toggleac(e);
     unsigned long timeout = millis() + 4000;  // Adjust as needed


    while (millis() < timeout) {
      if (toggleStateACOutput) {
        Serial2.write('m');
      } else {
        Serial2.write('n');
      }
      delay(10);
      // Read serial response
      if (Serial2.available() > 0) {
        String response = Serial2.readStringUntil('\n');
        if (response.equals("turnoffi") && toggleStateACOutput) {
          break;  // Break the loop if "turnoff" received and toggleState is true
        } else if (response.equals("turnoni") && !toggleStateACOutput) {
          break;  // Break the loop if "turnon" received and toggleState is false
        }
      }
      delay(200);
    }
    toggleStateACOutput = !toggleStateACOutput;
  }
}
void ui_event_DCOutput(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED) {
    _ui_state_modify(ui_Switch4, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
    _ui_checked_set_text_value(ui_Label7, target, "ON", "OFF");
    toggledc(e);
     unsigned long timeout = millis() + 4000;  // Adjust as needed


    while (millis() < timeout) {
      if (toggleStateDCOutput) {
        Serial2.write('a');
      } else {
        Serial2.write('b');
      }
      delay(10);
      // Read serial response
      if (Serial2.available() > 0) {
        String response = Serial2.readStringUntil('\n');
        if (response.equals("turnoffd") && toggleStateDCOutput) {
          break;  // Break the loop if "turnoff" received and toggleState is true
        } else if (response.equals("turnond") && !toggleStateDCOutput) {
          break;  // Break the loop if "turnon" received and toggleState is false
        }
      }
      delay(200);
    }
    toggleStateDCOutput = !toggleStateDCOutput;
  }
}
void ui_event_USBA(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED) {
    _ui_state_modify(ui_Switch5, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
    _ui_checked_set_text_value(ui_Label9, target, "ON", "OFF");
    toggleusba(e);
     unsigned long timeout = millis() + 4000;  // Adjust as needed


    while (millis() < timeout) {
      if (toggleStateUSBA) {
        Serial2.write('e');
      } else {
        Serial2.write('f');
      }
      delay(10);
      // Read serial response
      if (Serial2.available() > 0) {
        String response = Serial2.readStringUntil('\n');
        if (response.equals("turnoffa") && toggleStateUSBA) {
          break;  // Break the loop if "turnoff" received and toggleState is true
        } else if (response.equals("turnona") && !toggleStateUSBA) {
          break;  // Break the loop if "turnon" received and toggleState is false
        }
      }
      delay(200);
    }
    toggleStateUSBA = !toggleStateUSBA;
  }
}
void ui_event_Fan(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED) {
    _ui_state_modify(ui_Switch6, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
    _ui_checked_set_text_value(ui_Label11, target, "ON", "OFF");
    togglefan(e);
     unsigned long timeout = millis() + 4000;  // Adjust as needed


    while (millis() < timeout) {
      if (toggleStateFan) {
        Serial2.write('x');
      } else {
        Serial2.write('z');
      }
      delay(10);
      // Read serial response
      if (Serial2.available() > 0) {
        String response = Serial2.readStringUntil('\n');
        if (response.equals("turnofff") && toggleStateFan) {
          break;  // Break the loop if "turnoff" received and toggleState is true
        } else if (response.equals("turnonf") && !toggleStateFan) {
          break;  // Break the loop if "turnon" received and toggleState is false
        }
      }
      delay(200);
    }
    toggleStateFan = !toggleStateFan;
  }
}

void serial() {
  if (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');  // Read the serial data until newline character
    // Split the received data by commas
    int index = 0;
    String values[4];
    int startPos = 0;
    for (int i = 0; i < receivedData.length(); i++) {
      if (receivedData[i] == ',') {
        values[index] = receivedData.substring(startPos, i);
        startPos = i + 1;
        index++;
      }
    }
    values[index] = receivedData.substring(startPos);
    float temp = values[0].toFloat();
    int battery_level = values[3].toInt();
    float volt = values[1].toFloat();
    float amp = values[2].toFloat();
    int power = volt * abs(amp);
    // int power = volt * amp;
    if (amp < 0) {
      lv_obj_set_style_arc_color(ui_battery_indicator, lv_color_hex(0x12EB1C), LV_PART_INDICATOR | LV_STATE_DEFAULT);
      lv_label_set_text(ui_Time, "---");
    } else {
      lv_obj_set_style_arc_color(ui_battery_indicator, lv_color_hex(0x1FD2FC), LV_PART_INDICATOR | LV_STATE_DEFAULT);
      int batteryLife_minutes = calculateBatteryLife( battery_level, amp);
      if (batteryLife_minutes > 1000){
        lv_label_set_text(ui_Time, "-");
      }else{
        lv_label_set_text(ui_Time, String(batteryLife_minutes).c_str());
      }
      
    }

    lv_label_set_text(ui_Temp, String(temp, 1).c_str());
    lv_label_set_text(ui_Power, String(power).c_str());
    lv_label_set_text(ui_Volt, String(volt, 1).c_str());
    lv_label_set_text(ui_Amp, String(abs(amp)).c_str());
    // lv_label_set_text(ui_Amp, String(amp).c_str());
    lv_label_set_text(ui_battery_level, String(battery_level).c_str());
    lv_arc_set_value(ui_battery_indicator, battery_level);
    
  }
}

const float batteryCapacity_Ah = 30;
float calculateBatteryLife(float batteryLevel_percentage, float currentConsumption_A) {
  float batteryCurrent_A = (batteryLevel_percentage / 100) * batteryCapacity_Ah;
  return (batteryCurrent_A / currentConsumption_A) * 60; // Convert to minutes
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  serial();
  delay(5);
}
