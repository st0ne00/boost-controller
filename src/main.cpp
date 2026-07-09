#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MCP4725.h>

#include "Config.h"
#include "Flash.h"
#include "Rotary.h"
#include "Fase.h"
#include "Boost.h"
#include "Iat.h"

// PINOS
#define ROT_SW PB3
#define ROT_DT PA15
#define ROT_CK PB5
#define PHASE_PIN PB12
#define MAP_PIN PB1
#define WG_PIN PB0
#define LED_PIN PC13
#define THROTTLE_PIN PA2
#define IAT_PIN PA1
#define I2C3_SDA PB4
#define I2C3_SCL PA8
#define BT_PIN PA0

// DISPLAY
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C

#define PID_FREQ 1000
#define PID_PERIOD (1.0f / PID_FREQ)

#define RPM_MIN 750
#define RPM_MAX 6500
#define RPM_STEPS 250

#define IAT_RESISTOR 2160

// TELEMETRY
bool testing_mode = false;
uint32_t test_time = 0;
int test_delay = 5;

char status = 'n';

// DISPLAY
bool ledState = false;
int8_t display_page = 0;
uint32_t last_display_time = 0;
uint32_t last_led_time = 0;

TwoWire Wire3(I2C3_SDA, I2C3_SCL);
SPIClass spi1(FLASH_MOSI, FLASH_MISO, FLASH_SCK);
Flash flash(spi1, FLASH_CS);

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

HardwareTimer *timer;

HardwareSerial debug(PA3, PA2); // RX, TX
Rotary enc(ROT_SW, ROT_CK, ROT_DT);
Fase fase(PHASE_PIN);
Boost boost(PID_FREQ, 10, 10);
Iat iat(10);

Adafruit_MCP4725 dac;

enum DisplayPage {
  MAIN,
  IAT_TEST,
  CFG_LOAD,
  ADJ_PRESSAO,
  ADJ_MAPA,
  ADJ_KP,
  ADJ_KI,
  ADJ_KD,
  ADJ_INTEGRAL_LIMIT,
  ADJ_ERR_SPOOL,
  ADJ_ERR_PRE_PEAK,
  ADJ_CUT_ERR,
  ADJ_BASE_DUTY,
  ADJ_BASE_KPA,
  ADJ_MAP_CAL,
  CFG_SAVE,
  CFG_RESET
};

void enc_isr();
void sw_isr();
void phase_isr();
void control_isr();
unsigned int get_rpm_index(int rpm);

unsigned int get_rpm_index(int rpm)
{
  if (rpm > RPM_MAX)
  {
    rpm = RPM_MAX;
  }
  int index = (rpm - RPM_MIN) / RPM_STEPS;
  if (index > 26)
  {
    index = 26;
  }
  return index;
}

bool eeprom_ok = false;
void setup()
{
  pinMode(MAP_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(WG_PIN, OUTPUT);
  pinMode(BT_PIN, INPUT_PULLUP);

  debug.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  Wire3.begin();
  Wire3.setClock(400000);

  delay(200);

  // oled init
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(LED_BUILTIN, 1);
      delay(300);
      digitalWrite(LED_BUILTIN, 0);
      delay(300);
    }
  }

  // DAC init
  if(!dac.begin(0x60, &Wire3)) {
    for (int i = 0; i < 10; i++)
    {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.setCursor(0, 0);
      oled.println("DAC FAIL");
      oled.display();
      digitalWrite(LED_BUILTIN, 1);
      delay(100);
      oled.clearDisplay();
      oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      oled.setTextSize(2);
      oled.setCursor(0, 0);
      oled.println("DAC FAIL");
      oled.display();
      digitalWrite(LED_BUILTIN, 0);
      delay(100);
    }
  }

  // SPI flash init
  spi1.begin();
  if (!flash.init()) {
    for (int i = 0; i < 10; i++)
    {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.setCursor(0, 0);
      oled.println("FLASH FAIL");
      oled.display();
      digitalWrite(LED_BUILTIN, 1);
      delay(100);
      oled.clearDisplay();
      oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      oled.setTextSize(2);
      oled.setCursor(0, 0);
      oled.println("FLASH FAIL");
      oled.display();
      digitalWrite(LED_BUILTIN, 0);
      delay(100);
    }
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  oled.setCursor(0, 0);
  oled.display();

  enc.init();
  fase.init();

  timer = new HardwareTimer(TIM2);
  timer->setOverflow(PID_FREQ, HERTZ_FORMAT);
  timer->attachInterrupt(control_isr);
  timer->resume();

  attachInterrupt(digitalPinToInterrupt(ROT_CK), enc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROT_SW), sw_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PHASE_PIN), phase_isr, RISING);
}

int raw_map = 0;
int pa_map = 0;
int rpm = 0;
int raw_thr = 0;
int thr = 0;

int raw_iat = 0;
int v_iat = 0;

void loop()
{
  digitalWrite(LED_BUILTIN, ledState);

  // DISPLAY
  if ((millis() - last_display_time) > 30) // atualizar display a cada 30 ms
  {
    if (!digitalRead(PA0))
    {
      test_delay = 0;
      testing_mode = false;
      status = 'n';
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      display_page = 0;
    }
    oled.clearDisplay();
    oled.setCursor(0, 0);

    if (enc.hasClicked())
    {
      test_delay = 0;
      testing_mode = false;
      status = 'n';
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      display_page++;
    }

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
        oled.print(iat.get_data(Iat::IAT_INPUT_C));
        oled.setTextSize(1);
        oled.print("C");

        oled.setTextSize(1);
        //linha 2 col 1
        oled.setCursor(0, 16);
        oled.print("req ");
        oled.print((boost.get_data(Boost::REQ_PRESSURE) / 1000));

        //linha 2 col 2
        oled.setCursor(60, 16);
        oled.print("rpm ");
        oled.print(rpm);

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
      case IAT_TEST:
        oled.setTextSize(2);
        // linha 1
        oled.println("IAT TEST");
        // linha 2
        oled.setTextSize(1);
        oled.setCursor(0, 16);
        oled.print("mv in     = ");
        oled.println(iat.get_data(Iat::IAT_INPUT_MV));

        oled.print("mv out    = ");
        oled.println(iat.get_data(Iat::IAT_OUTPUT_MV));

        oled.print("IAT K in  = ");
        oled.println(iat.get_data(Iat::IAT_INPUT_K));
        // linha 3
        oled.print("IAT K out = ");
        oled.println(iat.get_data(Iat::IAT_OUTPUT_K));
        // linha 4
        oled.print("IAT C in = ");
        oled.println(iat.get_data(Iat::IAT_OUTPUT_C));

        oled.print("IAT C out = ");
        oled.println(iat.get_data(Iat::IAT_OUTPUT_C));
        
        break;
      case CFG_SAVE:
        oled.setTextSize(2);
        // linha 1
        oled.println("Salvar CFG");
        // linha 2
        oled.print("Flash ");
        if (flash.is_ok()) {
          oled.println("OK");
        } else {
          oled.println("ER");
        }
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            if(flash.save_config()) {
              oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
              oled.println("Cfg salva!");
            } else {
              oled.println("Cfg erro!");
            }
          }
          enc.reset();
        }
        break;
      case CFG_LOAD:
        oled.setTextSize(2);
        // linha 1
        oled.println("Load CFG");
        // linha 2
        oled.print("Flash ");
        if (flash.is_ok()) {
          oled.println("OK");
        } else {
          oled.println("ER");
        }
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            if(flash.read_config()) {
              oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
              oled.println("Cfg load!");
            } else {
              oled.println("Cfg erro!");
            }
          }
          enc.reset();
        }
        break;
      case CFG_RESET:
        oled.setTextSize(2);
        // linha 1
        oled.println("Reset CFG");
        
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::cfg_data = Cfg::cfg_default;
            oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            oled.println("Cfg Reset!");
          }
          enc.reset();
        }
        break;
      case ADJ_MAPA:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("MAPA");
        oled.print("table = ");
        oled.println(Cfg::selected_map);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            if(Cfg::selected_map + 1 > 2) {
              Cfg::selected_map = 2;
            } else {
              Cfg::selected_map += 1;
            }
          }
          else if (enc.getValue() < 0)
          {
            if(Cfg::selected_map - 1 < 0) {
              Cfg::selected_map = 0;
            } else {
              Cfg::selected_map -= 1;
            }
          }
          enc.reset();
        }
        break;
      case ADJ_KP:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj PID");
        oled.println("const");
        oled.print("P = ");
        oled.println(Cfg::kp);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::kp += 5;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::kp -= 5;
          }
          enc.reset();
        }
        break;
      case ADJ_KI:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj PID");
        oled.println("const");
        oled.print("I = ");
        oled.println(Cfg::ki);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::ki += 5;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::ki -= 5;
          }
          enc.reset();
        }
        break;
      case ADJ_KD:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj PID");
        oled.println("const");
        oled.print("D = ");
        oled.println(Cfg::kd);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::kd += 5;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::kd -= 5;
          }
          enc.reset();
        }
        break;
      case ADJ_ERR_SPOOL:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj ERR");
        oled.println("spool end");
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
      case ADJ_ERR_PRE_PEAK:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj ERR");
        oled.println("p-peak end");
        oled.print("E = ");
        oled.println(Cfg::err_pre_peak_end);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::err_pre_peak_end += 1000;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::err_pre_peak_end -= 1000;
          }
          enc.reset();
        }
        break;
      case ADJ_INTEGRAL_LIMIT:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj PID");
        oled.println("integral");
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
        oled.println("Adj BASE");
        oled.println("");
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
        oled.println("Adj BASE");
        oled.println("");
        oled.print("kPa = ");
        oled.println(Cfg::base_kpa);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::base_kpa += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::base_kpa -= 1;
          }
          enc.reset();
        }
        break;
      case ADJ_CUT_ERR:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj CUT");
        oled.println("");
        oled.println("atm + ");
        oled.println(Cfg::cut_threshold);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::cut_threshold += 1000;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::cut_threshold -= 1000;
          }
          enc.reset();
        }
        break;
      case ADJ_MAP_CAL:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Adj MAP");
        oled.print("cal = ");
        oled.println(Cfg::map_cal);
        oled.print("Raw: ");
        oled.println(raw_map);
        oled.print("Pa: ");
        oled.println(pa_map);
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
      case ADJ_PRESSAO:
        oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        oled.setTextSize(2);
        oled.println("Max kPa");
        oled.println("");
        oled.print("P = ");
        oled.println(Cfg::set_pressure);
        if (enc.hasMoved())
        {
          if (enc.getValue() > 0)
          {
            Cfg::set_pressure += 1;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::set_pressure -= 1;
          }
          enc.reset();
        }
        break;
    }

    oled.display();
    last_display_time = millis();
  }
}

// INTERRUPTS

void control_isr() { // rodando a cada 1 ms
  raw_map = analogRead(MAP_PIN);
  pa_map = raw_map * Cfg::map_cal; // pascal

  raw_thr = analogRead(THROTTLE_PIN);
  thr = (raw_thr * 100) / Cfg::thr_cal; // % pedal acelerador

  raw_iat = analogRead(IAT_PIN);
  v_iat = (raw_iat * 3232) / 1000; // converter para mV

  rpm = fase.getRPM();
  int rpm_index = get_rpm_index(rpm);

  boost.update_boost(pa_map);
  boost.loop(rpm, rpm_index, thr);
  analogWrite(WG_PIN, (boost.get_data(Boost::DUTY) * 255) / 100);

  iat.update(v_iat);
  iat.loop(rpm, rpm_index, (boost.get_data(Boost::BOOST)/1000));
  dac.setVoltage(iat.get_dac_value(), false);

}

void phase_isr()
{
  fase.pulseISR();
}

void enc_isr()
{
  enc.poll();
}

void sw_isr()
{
  enc.pollSW();
}