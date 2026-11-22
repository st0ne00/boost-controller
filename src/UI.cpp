#include "UI.h"
#include "Config.h"
#include <string.h>

// Add entries to these tables to extend either menu level.
const UI::Entry UI::pressureEntries[] = {
  {"Base Press", ADJ_BASE_KPA}, {"Err Spool", ADJ_ERR_SPOOL},
  {"Err Cut", ADJ_CUT_ERR}, {"Map Cal", ADJ_MAP_CAL}, {"Boost Map", BOOST_TABLE}
};
const UI::Entry UI::dutyEntries[] = {
  {"Base Duty", ADJ_BASE_DUTY}, {"Idle Duty", UNAVAILABLE},
  {"Max Duty", UNAVAILABLE}, {"Min Duty", UNAVAILABLE}, {"Duty Map", DUTY_TABLE}
};
const UI::Entry UI::pidEntries[] = {
  {"Const P", ADJ_KP}, {"Const I", ADJ_KI}, {"Const D", ADJ_KD},
  {"Max Integral", ADJ_INTEGRAL_LIMIT}
};
const UI::Entry UI::throttleEntries[] = {
  {"Max Thr", UNAVAILABLE}, {"Min Thr", UNAVAILABLE},
  {"Thr Cal", UNAVAILABLE}, {"Idle RPM", UNAVAILABLE}
};
const UI::Entry UI::mapEntries[] = {
  {"Salvar", MAP_SAVE}, {"Carregar", MAP_LOAD}, {"Reset", MAP_RESET}
};
const UI::Entry UI::configEntries[] = {
  {"Salvar", CFG_SAVE}, {"Carregar", CFG_LOAD}, {"Reset", CFG_RESET}
};
#define ENTRY_COUNT(a) (sizeof(a) / sizeof((a)[0]))
const UI::Category UI::categories[] = {
  {"Pressure", pressureEntries, ENTRY_COUNT(pressureEntries)},
  {"Duty", dutyEntries, ENTRY_COUNT(dutyEntries)},
  {"PID", pidEntries, ENTRY_COUNT(pidEntries)},
  {"Acelerador", throttleEntries, ENTRY_COUNT(throttleEntries)},
  {"Cfg Mem", configEntries, ENTRY_COUNT(configEntries)},
  {"Map Mem", mapEntries, ENTRY_COUNT(mapEntries)}
};
const uint8_t UI::categoryCount = ENTRY_COUNT(categories);
#undef ENTRY_COUNT

void UI::title(const char* text)
{
  if (text == nullptr) {
    if (screen == Screen::Categories) text = "Menu";
    else if (screen == Screen::Entries) text = categories[categoryIndex].label;
    else if (screen == Screen::Page) text = categories[categoryIndex].entries[entryIndex].label;
    else text = "Dashboard";
  }
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  oled.setCursor(0, 0);
  oled.print(text);
  oled.fillRect(0, 10, 128, 2, SSD1306_WHITE);
  oled.setCursor(0, 16);
}

void UI::drawMenu()
{
  const bool top = screen == Screen::Categories;
  const uint8_t selected = top ? categoryIndex : entryIndex;
  const uint8_t count = top ? categoryCount : categories[categoryIndex].count;
  uint8_t& scroll = top ? categoryScroll : entryScroll;
  constexpr uint8_t visible = 3;
  if (selected < scroll) scroll = selected;
  if (selected >= scroll + visible) scroll = selected - visible + 1;
  title();
  for (uint8_t row = 0; row < visible && scroll + row < count; ++row) {
    const uint8_t index = scroll + row;
    const char* label = top ? categories[index].label : categories[categoryIndex].entries[index].label;
    oled.setTextSize(strlen(label) > 10 ? 1 : 2);
    if (index == selected) oled.fillRect(0, 16 + row * 16, 128, 17, SSD1306_WHITE);
    oled.setCursor(1, 17 + row * 16);
    oled.setTextColor(index == selected ? SSD1306_BLACK : SSD1306_WHITE,
                      index == selected ? SSD1306_WHITE : SSD1306_BLACK);
    oled.print(label);
    if (!top && index == selected && actionResult != ActionResult::None) {
      oled.setTextSize(2);
      oled.setCursor(104, 17 + row * 16);
      oled.print(actionResult == ActionResult::Ok ? "OK" : "ER");
    }
  }
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
}

bool UI::runAction(DisplayPage action, bool& ok)
{
  ok = true;
  switch (action) {
    case CFG_SAVE:
      ok = flash.save(Cfg::cfg_data, Cfg::cfg_address);
      break;
    case CFG_LOAD:
      ok = flash.load(Cfg::cfg_data, Cfg::cfg_address);
      break;
    case CFG_RESET:
      Cfg::cfg_data = Cfg::cfg_default;
      break;
    case MAP_SAVE: {
      const bool boost_ok = flash.save(Cfg::boost_rpm_mul, Cfg::boost_map_address);
      const bool duty_ok = flash.save(Cfg::duty_rpm_mul, Cfg::duty_map_address);
      ok = boost_ok && duty_ok;
      break;
    }
    case MAP_LOAD: {
      auto boost_map = Cfg::boost_rpm_mul;
      auto duty_map = Cfg::duty_rpm_mul;
      ok = flash.load(boost_map, Cfg::boost_map_address) &&
           flash.load(duty_map, Cfg::duty_map_address);
      if (ok) {
        Cfg::boost_rpm_mul = boost_map;
        Cfg::duty_rpm_mul = duty_map;
      }
      break;
    }
    case MAP_RESET:
      Cfg::boost_rpm_mul = Cfg::boost_rpm_default;
      Cfg::duty_rpm_mul = Cfg::duty_rpm_default;
      break;
    default:
      return false;
  }
  return true;
}

void UI::drawMapEditor(std::array<int, Cfg::rpm_steps>& map, bool pressure, bool clicked)
{
  if (clicked) mapValueSelected = !mapValueSelected;
  const int movement = enc.getValue(true);
  if (!mapValueSelected) {
    int64_t next = static_cast<int64_t>(mapIndex) + movement;
    if (next < 0) next = 0;
    if (next >= Cfg::rpm_steps) next = Cfg::rpm_steps - 1;
    mapIndex = static_cast<int>(next);
  } else if (movement != 0 && (!pressure || Cfg::base_pressure > 0)) {
    int64_t next;
    if (pressure) {
      // The boost map stores a multiplier in thousandths, not mbar.
      const int64_t current = static_cast<int64_t>(Cfg::base_pressure) * map[mapIndex] / 1000;
      const int64_t requested = current + static_cast<int64_t>(movement) * 2;
      next = (requested * 1000 + Cfg::base_pressure / 2) / Cfg::base_pressure;
    } else {
      next = static_cast<int64_t>(map[mapIndex]) + movement;
    }
    // Keep the displayed multiplier and the controller's integer arithmetic bounded.
    if (next < 0) next = 0;
    if (next > 9999) next = 9999;
    map[mapIndex] = static_cast<int>(next);
  }

  oled.setTextSize(2);
  oled.setCursor(0, 16);
  oled.setTextColor(mapValueSelected ? SSD1306_WHITE : SSD1306_BLACK,
                    mapValueSelected ? SSD1306_BLACK : SSD1306_WHITE);
  oled.print(Cfg::rpm_min + mapIndex * Cfg::rpm_step);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  oled.print(" rpm");
  oled.setCursor(0, 40);
  oled.setTextColor(mapValueSelected ? SSD1306_BLACK : SSD1306_WHITE,
                    mapValueSelected ? SSD1306_WHITE : SSD1306_BLACK);
  if (pressure && Cfg::base_pressure <= 0) {
    oled.print("N/A");
  } else if (pressure) {
    oled.print(static_cast<int32_t>(static_cast<int64_t>(Cfg::base_pressure) * map[mapIndex] / 1000));
  } else {
    oled.print(map[mapIndex]);
  }
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  oled.setTextSize(2);
  oled.print(pressure ? " mbar" : " %");
}

UI::UI(Adafruit_SSD1306& display, Rotary& encoder, Boost& controller, Flash& storage)
  : oled(display), enc(encoder), boost(controller), flash(storage)
{
}

void UI::update(int rpm, int raw_map, int mbar_map, bool reset_requested)
{
  // DISPLAY
  if ((millis() - last_display_time) > 30) // atualizar display a cada 30 ms
  {
    const bool clicked = enc.hasClicked();
    const bool held = enc.hasHeld();
    const bool pageClicked = screen == Screen::Page && clicked && !held && !reset_requested;
    if (reset_requested) {
      actionResult = ActionResult::None;
      screen = Screen::Dashboard;
      display_page = MAIN;
      enc.reset();
    } else if (held) {
      actionResult = ActionResult::None;
      if (screen == Screen::Page) screen = Screen::Entries;
      else if (screen == Screen::Entries) screen = Screen::Categories;
      else if (screen == Screen::Categories) screen = Screen::Dashboard;
      enc.reset();
    } else if (clicked && screen != Screen::Page) {
      if (screen == Screen::Dashboard) screen = Screen::Categories;
      else if (screen == Screen::Categories) {
        screen = Screen::Entries;
        entryIndex = 0;
        entryScroll = 0;
      } else {
        const DisplayPage selectedPage = categories[categoryIndex].entries[entryIndex].page;
        bool ok;
        if (runAction(selectedPage, ok)) {
          actionResult = ok ? ActionResult::Ok : ActionResult::Error;
        } else {
          actionResult = ActionResult::None;
          display_page = selectedPage;
          screen = Screen::Page;
          mapIndex = 0;
          mapValueSelected = false;
        }
      }
      enc.reset();
    }

    if (screen == Screen::Categories || screen == Screen::Entries) {
      const int movement = enc.getValue(true);
      uint8_t& selected = screen == Screen::Categories ? categoryIndex : entryIndex;
      const int count = screen == Screen::Categories ? categoryCount : categories[categoryIndex].count;
      int next = static_cast<int>(selected) + movement;
      if (next < 0) next = 0;
      if (next >= count) next = count - 1;
      if (next != selected) actionResult = ActionResult::None;
      selected = static_cast<uint8_t>(next);
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    oled.setCursor(0, 0);
    if (screen == Screen::Categories || screen == Screen::Entries) {
      drawMenu();
      oled.display();
      last_display_time = millis();
      return;
    }
    if (screen == Screen::Dashboard) display_page = MAIN;
    else title();

    switch(display_page) {
      default:
        display_page = MAIN;
      case MAIN:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        //linha 1 col 1
        oled.setCursor(0, 0);
        oled.setTextSize(2);
        oled.print((boost.get_data(Boost::ABS_PRESSURE) / 1000));
        oled.setTextSize(1);
        oled.print("kPa");

        //linha 1 col 2
        oled.setCursor(62, 0);
        oled.setTextSize(2);
        oled.print(rpm);
        oled.setTextSize(1);
        oled.print("rpm");

        oled.setTextSize(1);
        //linha 2 col 1
        oled.setCursor(0, 16);
        oled.print("req ");
        oled.print((boost.get_data(Boost::REQ_PRESSURE) / 1000));

        //linha 2 col 2
        oled.setCursor(60, 16);
        //oled.print("rpm ");
        //oled.print(rpm);

        //linha 3 col 1
        oled.setCursor(0, 24);
        oled.print("p ");
        oled.print(boost.get_data(Boost::PID_P));

        //linha 3 col 2
        oled.setCursor(60, 24);
        oled.print("i ");
        oled.print(boost.get_data(Boost::PID_I));

        //linha 4 col 1
        oled.setCursor(0, 32);
        oled.print("d ");
        oled.print(boost.get_data(Boost::PID_D));

        //linha 4 col 2
        oled.setCursor(60, 32);
        oled.print("pid ");
        oled.print(boost.get_data(Boost::PID));

        //linha 5 col 1
        oled.setCursor(0, 40);
        oled.print("err ");
        oled.print(boost.get_data(Boost::ERROR));

        //linha 5 col 2
        oled.setCursor(60, 40);
        oled.print("atm ");
        oled.print(boost.get_data(Boost::ATM_PRESSURE));

        //linha 6 col 1
        oled.setTextSize(2);
        oled.setCursor(0, 48);
        switch(boost.get_data(Boost::STATE)) {
          default:
            oled.print("ERROR");
            break;
          case Boost::IDLE:
            oled.print("IDLE");
            break;
          case Boost::SPOOL:
            oled.print("SPOOL");
            break;
          case Boost::PEAK:
            oled.print("PEAK");
            break;
          case Boost::MESA:
            oled.print("MESA");
            break;
          case Boost::CUT:
            oled.print("CUT");
            break;
        }

        //linha 6 col 2
        oled.setCursor(90, 48);
        oled.print(boost.get_data(Boost::DUTY));
        oled.print("%");
        break;
      case UNAVAILABLE:
        oled.setTextSize(2);
        oled.println("Not yet");
        oled.println("available");
        if (enc.hasMoved()) enc.reset();
        break;
      case BOOST_TABLE:
        drawMapEditor(Cfg::boost_rpm_mul, true, pageClicked);
        break;
      case DUTY_TABLE:
        drawMapEditor(Cfg::duty_rpm_mul, false, pageClicked);
        break;
      case ADJ_KP:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("P = ");
        oled.println(Cfg::kp);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::kp += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::kp -= 1;
          }
          enc.reset();
        }
        break;
      case ADJ_KI:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("I = ");
        oled.println(Cfg::ki);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::ki += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::ki -= 1;
          }
          enc.reset();
        }
        break;
      case ADJ_KD:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("D = ");
        oled.println(Cfg::kd);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::kd += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::kd -= 1;
          }
          enc.reset();
        }
        break;
      case ADJ_ERR_SPOOL:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("E = ");
        oled.println(Cfg::err_spool_end);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::err_spool_end += 1000;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::err_spool_end -= 1000;
          }
          enc.reset();
        }
        break;
      case ADJ_INTEGRAL_LIMIT:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("limit = ");
        oled.println(Cfg::integral_limit);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::integral_limit += 50000;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::integral_limit -= 50000;
          }
          enc.reset();
        }
        break;
      case ADJ_BASE_DUTY:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("duty = ");
        oled.println(Cfg::base_duty);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::base_duty += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::base_duty -= 1;
          }
          enc.reset();
        }
        break;
      case ADJ_BASE_KPA:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("kPa = ");
        oled.println(Cfg::base_pressure);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::base_pressure += 5;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::base_pressure -= 5;
          }
          enc.reset();
        }
        break;
      case ADJ_CUT_ERR:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("atm + ");
        oled.println(Cfg::err_cut);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::err_cut += 10;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::err_cut -= 10;
          }
          enc.reset();
        }
        break;
      case ADJ_MAP_CAL:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.print("cal = ");
        oled.println(Cfg::map_cal);
        oled.print("Raw: ");
        oled.println(raw_map);
        oled.print("Pa: ");
        oled.println(mbar_map);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::map_cal += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::map_cal -= 1;
          }
          enc.reset();
        }
        break;
    }

    oled.display();
    last_display_time = millis();
  }
}
