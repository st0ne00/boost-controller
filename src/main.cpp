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
uint16_t map_cal = 278; // map_cal x1000
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
  ADJ_KD
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
            Cfg::kp += 10;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::kp -= 10;
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
            Cfg::ki += 10;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::ki -= 10;
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
            Cfg::kd += 10;
          }
          else if (enc.getValue() < 0)
          {
            Cfg::kd -= 10;
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
  int map = raw_map * map_cal; // pascal
  rpm = fase.getRPM();
  int rpm_index = get_rpm_index(rpm);
  boost.update_boost(map);
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



/*

    switch (display_page)
    {
    default:
      display_page = 0;
    case 0:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);

      oled.print(map_value);
      oled.setTextSize(1);
      oled.print("KPA");

      oled.setCursor(62, 0);
      oled.setTextSize(2);
      //oled.print(rpm);
      oled.setTextSize(1);
      oled.print("RPM");

      oled.setTextSize(1);
      oled.setCursor(0, 16);
      oled.print("pk ");
      oled.print(peakMap);
      oled.setCursor(80, 16);
      oled.print("err  ");
      oled.print(boost_error);

      oled.setCursor(0, 24);
      oled.print("int ");
      oled.print(integral_error);
      oled.setCursor(65, 24);
      oled.print("io ");
      oled.print(i_out);

      oled.setCursor(0, 32);
      oled.print("po ");
      oled.print(p_out);
      oled.setCursor(55, 32);
      oled.print("over ");
      oled.print(overboost_count);

      oled.setCursor(0, 40);
      oled.print("pk st ");
      oled.println(peak_state);
      oled.setCursor(55, 40);
      //oled.print("L ");
      //oled.println(loop_counter);

      oled.setCursor(0, 48);
      oled.setTextSize(2);
      if (status != 'e')
      {
        switch (state)
        {
        case IDLE:
          oled.print("IDLE ");
          break;
        case SPOOL:
          oled.print("SPOOL");
          break;
        case PEAK:
          oled.print("PEAK ");
          break;
        case MESA:
          oled.print("MESA ");
          break;
        case CUT:
          oled.print("CUT  ");
          break;
        }
      }
      else
      {
        oled.print("ERR  ");
      }

      oled.print("  ");
      oled.print(duty);
      oled.print("%");
      enc.reset();
      break;
    case 1:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("SEL. MAPA");
      oled.print("mapa = ");
      oled.println(map_num);
      //oled.print(mapa[map_num][10]);
      oled.println(" kpa");
      oled.println("@ 3000 RPM");
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          map_num++;
        }
        else if (enc.getValue() < 0)
        {
          map_num--;
        }
        enc.reset();
        if (map_num > NUM_MAPAS)
        {
          map_num = NUM_MAPAS;
        }
        else if (map_num < 0)
        {
          map_num = 0;
        }
      }
      break;
    case 2:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("CURVA DUTY");
      oled.println("antes do 0");
      oled.print("K = ");
      oled.println(k_duty_under);
      // oled.print(mapa[map_num][10]);
      // oled.println(" kpa");
      // oled.println("@ 3000 RPM");
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          k_duty_under++;
        }
        else if (enc.getValue() < 0)
        {
          k_duty_under--;
        }
        enc.reset();
      }
      break;
    case 3:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("CURVA DUTY");
      oled.println("apos o 0");
      oled.print("K = ");
      oled.println(k_duty_over);
      // oled.print(mapa[map_num][10]);
      // oled.println(" kpa");
      // oled.println("@ 3000 RPM");
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          k_duty_over++;
        }
        else if (enc.getValue() < 0)
        {
          k_duty_over--;
        }
        enc.reset();
      }
      break;
    case 4:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("CURVA DUTY");
      oled.println("mesa 0 =");
      oled.print(err_base_start);
      oled.print(" <-> ");
      oled.println(err_base_end);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          err_base_start++;
          err_base_end--;
        }
        else if (enc.getValue() < 0)
        {
          err_base_end++;
          err_base_start--;
        }
        enc.reset();
        if (err_base_start < 1)
        {
          err_base_start = 1;
          err_base_end = -1;
        }
      }
      break;
    case 5:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("SPOOL CFG");
      oled.println("ending err");
      oled.print("err = ");
      oled.println(spool_end_error);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          spool_end_error++;
        }
        else if (enc.getValue() < 0)
        {
          spool_end_error--;
        }
        enc.reset();
      }
      break;
    case 6: // MULTIPLICADOR SPOOL ( antes do 0 )
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PEAK CFG");
      oled.println("spool mul");
      oled.print("S = ");
      oled.println(peak_spool_mul);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          peak_spool_mul++;
        }
        else if (enc.getValue() < 0)
        {
          peak_spool_mul--;
        }
        enc.reset();
      }
      break;
    case 7: // MULTIPLICADOR INÍCIO PICO ( após o 0 e antes do pico )
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PEAK CFG");
      oled.println("over mul");
      oled.print("O = ");
      oled.println(peak_over_mul);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          peak_over_mul++;
        }
        else if (enc.getValue() < 0)
        {
          peak_over_mul--;
        }
        enc.reset();
      }
      break;
    case 8: // MULTIPLICADOR FIM PICO ( após pico e antes do 0 )
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PEAK CFG");
      oled.println("end mul");
      oled.print("E = ");
      oled.println(peak_end_mul);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          peak_end_mul++;
        }
        else if (enc.getValue() < 0)
        {
          peak_end_mul--;
        }
        enc.reset();
      }
      break;
    case 9:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PEAK CFG");
      oled.println("ending err");
      oled.print("err = ");
      oled.println(peak_end_error);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          peak_end_error++;
        }
        else if (enc.getValue() < 0)
        {
          peak_end_error--;
        }
        enc.reset();
      }
      break;

    case 10: // TIMEOUT PICO
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PEAK CFG");

      oled.println("timeout");
      oled.print("t = ");
      oled.println(peak_timeout);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          peak_timeout = peak_timeout + 50;
        }
        else if (enc.getValue() < 0)
        {
          peak_timeout = peak_timeout - 50;
        }
        enc.reset();
      }
      break;
    case 11:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("CUT ERR");

      oled.print("e = ");
      oled.println(cut_error);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          cut_error++;
        }
        else if (enc.getValue() < 0)
        {
          cut_error--;
        }
        enc.reset();
      }
      break;
    case 12:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PID CONFIG");
      oled.println("constante");
      oled.print("KP = ");
      oled.println(pid_kp);
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          pid_kp = pid_kp + 5;
        }
        else if (enc.getValue() < 0)
        {
          if (pid_kp > 0)
          {
            pid_kp = pid_kp - 5;
          }
          else
          {
            pid_kp = 0;
          }
        }
        enc.reset();
      }
      break;
    case 13:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PID CONFIG");
      oled.println("constante");
      oled.print("KI = ");
      oled.println(pid_ki);
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          pid_ki = pid_ki + 5;
        }
        else if (enc.getValue() < 0)
        {
          if (pid_ki > 0)
          {
            pid_ki = pid_ki - 5;
          }
          else
          {
            pid_ki = 0;
          }
        }
        enc.reset();
      }
      break;
    case 14:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PID CONFIG");
      oled.println("max integ.");
      oled.println("Imax = ");
      oled.println(integral_limit);
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          integral_limit = integral_limit + 100;
        }
        else if (enc.getValue() < 0)
        {
          if (integral_limit > 0)
          {
            integral_limit = integral_limit - 100;
          }
          else
          {
            integral_limit = 0;
          }
        }
        enc.reset();
      }
      break;
    case 15:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("BASE DUTY");
      oled.println("kpa ref");
      oled.print("B = ");
      oled.println(base_kpa);
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          base_kpa = base_kpa + 1;
        }
        else if (enc.getValue() < 0)
        {
          base_kpa = base_kpa - 1;
        }
        enc.reset();
      }
      break;
    case 16:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("BASE DUTY");
      oled.println("% @ base");
      oled.print("D = ");
      oled.println(base_duty);
      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          base_duty++;
        }
        else if (enc.getValue() < 0)
        {
          base_duty--;
        }
        enc.reset();
      }
      break;
    case 17:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("CALIB. MAP");

      oled.print("RAW: ");
      oled.println(raw_map);

      oled.print("KPA: ");
      oled.println(map_value);

      oled.print("C = ");
      oled.println(map_cal);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          map_cal = map_cal + 1;
        }
        else if (enc.getValue() < 0)
        {
          map_cal = map_cal - 1;
        }
        enc.reset();
      }
      break;
    case 18:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("OVER TIME");

      oled.println("max = ");
      oled.println(overboost_max_time);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          overboost_max_time++;
        }
        else if (enc.getValue() < 0)
        {
          overboost_max_time--;
        }
        enc.reset();
      }
      break;
    case 19:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("teste PWM");

      if (testing_mode)
      {
        oled.print("ativo: ");
        oled.println(test_delay);
      }
      else
      {
        oled.println("desativado");
      }
      oled.print("duty: ");
      oled.println(duty);

      oled.print("pwm: ");
      oled.println((duty * 255) / 100);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          test_delay = test_delay + 1;
          testing_mode = true;
          status = 't';
        }
        else if (enc.getValue() < 0)
        {
          test_delay = test_delay - 1;
          if (test_delay < 1)
          {
            test_delay = 0;
            testing_mode = false;
            status = 'n';
          }
        }
        enc.reset();
      }
      break;
    case 20:
      oled.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      oled.setTextSize(2);
      oled.println("PWM manual");

      if (testing_mode)
      {
        oled.println("ativo");
        // oled.println(test_delay);
      }
      else
      {
        oled.println("desativado");
      }
      oled.print("duty: ");
      oled.println(duty);

      oled.print("pwm: ");
      oled.println((duty * 255) / 100);

      if (enc.hasMoved())
      {
        if (enc.getValue() > 0)
        {
          if (status != 'm')
          {
            duty = 0;
            testing_mode = true;
          }
          status = 'm';
          duty = duty + 1;
        }
        else if (enc.getValue() < 0)
        {
          duty = duty - 1;
          if (duty < 0)
          {
            duty = 0;
            testing_mode = false;
            status = 'n';
          }
        }
        enc.reset();
      }
      break;
    }

*/