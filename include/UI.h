#ifndef UI_H
#define UI_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "Boost.h"
#include "Flash.h"
#include "Rotary.h"

class UI {
public:
  UI(Adafruit_SSD1306& display, Rotary& encoder, Boost& controller, Flash& storage);
  void update(bool reset_requested, int rpm, int raw_map, int mbar_map, int raw_thr, int thr);

private:
  enum DisplayPage {
    MAIN, UNAVAILABLE, CFG_LOAD, ADJ_KP, ADJ_KI, ADJ_KD,
    ADJ_INTEGRAL_LIMIT, ADJ_ERR_SPOOL, ADJ_CUT_ERR,
    ADJ_BASE_DUTY, ADJ_BASE_KPA, ADJ_MAP_CAL, CFG_SAVE, CFG_RESET,
    BOOST_TABLE, DUTY_TABLE, MAP_SAVE, MAP_LOAD, MAP_RESET, ADJ_IDLE_DUTY, ADJ_MAX_DUTY, ADJ_MIN_DUTY, ADJ_IDLE_RPM, ADJ_MAX_THR, ADJ_MIN_THR, ADJ_THR_CAL, DASH,
    ADJ_REFRESH_TIME
  };
  enum class Screen { Dashboard, Categories, Entries, Page };
  struct Entry { const char* label; DisplayPage page; };
  struct Category { const char* label; const Entry* entries; uint8_t count; };
  static const Entry pressureEntries[], dutyEntries[], pidEntries[], throttleEntries[], configEntries[], mapEntries[];
  static const Category categories[];
  static const uint8_t categoryCount;
  void title(const char* text = nullptr);
  void drawMenu();
  bool runAction(DisplayPage action, bool& ok);
  enum class ActionResult { None, Ok, Error };
  ActionResult actionResult = ActionResult::None;
  void drawMapEditor(std::array<int, Cfg::rpm_steps>& map, bool pressure, bool clicked);
  void drawValueEditor(int &value, int step, const char* unit, const char* desc);
  void drawBarGraph(int32_t value, int32_t min, int32_t max, int32_t start_y, int32_t height, const char* label, bool show_min_max, int32_t min_value, int32_t max_value);
  int mapIndex = 0;
  bool mapValueSelected = false;
  Screen screen = Screen::Dashboard;
  uint8_t categoryIndex = 0;
  uint8_t entryIndex = 0;
  uint8_t categoryScroll = 0;
  uint8_t entryScroll = 0;
  Adafruit_SSD1306& oled;
  Rotary& enc;
  Boost& boost;
  Flash& flash;
  int8_t display_page = MAIN;
  uint32_t last_display_time = 0;
};

#endif
