#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include "Config.h"
#include "Rotary.h"
#include "Fase.h"
#include "Boost.h"

// PINOS
#define ROT_SW PB3
#define ROT_DT PB4
#define ROT_CK PB5
#define PHASE_PIN PB12
#define MAP_PIN PB1
#define WG_PIN PA7
#define LED_PIN PC13

// #define map_cal (100.0f / 391.0f)
#define BT_PIN PA0

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C

#define PWM_MIN 0
#define PWM_MAX 80

#define DUTY_MAX 80
#define DUTY_MIN 20

#define PID_FREQ 1000
#define PID_PERIOD (1.0f / PID_FREQ)

#define MAP_MAX 250 // 237
#define MAP_MIN 40

/*
#define IDLE 0
#define SPOOL 1
#define PEAK 2
#define MESA 3
#define CUT 4
*/

#define RPM_MIN 750
#define RPM_MAX 6500
#define RPM_STEPS 250

// TELEMETRY
uint32_t peakMap = 0;
uint32_t peakRPM = 0;
bool testing_mode = false;
uint32_t test_time = 0;
int test_delay = 5;

// BOOST CONTROL
// config
int map_num = 1;
//uint16_t map_cal = 278; // map_cal x1000
int spool_end_error = 5;
int cut_rate_threshold = 100;
// int peak_start_error = 10;
// int peak_mid_error = -10;
// int peak_end_error = -5;
int cut_error = 40;
int peak_timeout = 2000;
// int base_kpa = 210;
int peak_spool_mul = 90;
int peak_over_mul = 60;
int peak_end_mul = 100;
int peak_end_error = -5;
int duty_table_selector = 0;

int base_kpa = 210;
int base_duty = 66;
int err_base_start = 10;
int err_base_end = -10;
int k_duty_under = 50;        // x100
int k_duty_over = 50;         // x100
int overboost_max_time = 200; // 50;

int32_t boost_error = 0;
uint32_t boost_req = 0;
uint32_t map_value = 0;
int duty = 0;
char status = 'n';
uint32_t overboost_count = 0;
uint32_t peak_start_time = 0;
//uint8_t state = IDLE;
int last_boost_error = 0;
int error_change_rate = 0;
int peak_state = 0;
int cut_time = 0;
int peak_duty_mul = 0;
int pk_last_error = 0;
int peak_error = 0;
unsigned int overboost_time = 0;
int overboost_status = 0;
unsigned int map_sum = 0;

// PID
int pid_kp = 50; // 100; // kp*100;
int pid_ki = 20; // 100; // ki*100;
int integral_error = 0;
int8_t pid_enable_err = 10;
int8_t pid_disable_err = 20;
int p_out = 0;
int i_out = 0;
bool i_enabled = false;
int integral_limit = 20000;

// DISPLAY
bool ledState = false;
int8_t display_page = 0;
uint32_t last_display_time = 0;
uint32_t last_led_time = 0;

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

HardwareTimer *timer;

HardwareSerial debug(PA3, PA2); // RX, TX
Rotary enc(ROT_SW, ROT_CK, ROT_DT);
Fase fase(PHASE_PIN);
Boost boost(PID_FREQ, 10, 10);

enum DisplayPage {
  MAIN,
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
};

void enc_isr();
void sw_isr();
void phase_isr();
void control_isr();
unsigned int get_rpm_index(int rpm);
unsigned int get_error_index(int error);
// int calc_duty(int rpm_index, int err_index, int base_kpa, int boost_req);
int calc_duty(int _rpm_index, int _err, int _boost_req);

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

unsigned int get_error_index(int error)
{
  int index = 30 - error;
  if (index > 60)
  {
    index = 60;
  }
  else if (index < 0)
  {
    index = 0;
  }
  return index;
}

void setup()
{
  debug.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(MAP_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(WG_PIN, OUTPUT);

  pinMode(BT_PIN, INPUT_PULLUP);

  delay(200);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(LED_BUILTIN, 1);
      delay(100);
      digitalWrite(LED_BUILTIN, 0);
      delay(100);
    }
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
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
      display_page = 0;
    }
    oled.clearDisplay();
    oled.setCursor(0, 0);

    if (enc.hasClicked())
    {
      test_delay = 0;
      testing_mode = false;
      status = 'n';
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
        oled.print("base ");
        oled.print(boost.get_data(Boost::BASE));

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
    }

    oled.display();
    last_display_time = millis();
  }
}

// INTERRUPTS

void control_isr() { // rodando a cada 1 ms
  raw_map = analogRead(MAP_PIN);
  pa_map = raw_map * Cfg::map_cal; // pascal
  rpm = fase.getRPM();
  int rpm_index = get_rpm_index(rpm);
  boost.update_boost(pa_map);
  boost.loop(rpm, rpm_index, 100);
  analogWrite(WG_PIN, (boost.get_data(Boost::DUTY) * 255) / 100);
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