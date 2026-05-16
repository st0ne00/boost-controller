#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

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

#define IDLE 0
#define SPOOL 1
#define PEAK 2
#define MESA 3
#define CUT 4

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
uint8_t state = IDLE;
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

// unsigned int rpm_table[26] = {750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000};

// multiplicador de duty cycle em relação ao RPM, considerando 100% @ 4000RPM
unsigned int rpm_duty_mul[26] = {83, 84, 86, 88, 91, 92, 94, 96, 96, 96, 96, 97, 97, 100, 102, 104, 108, 112, 118, 121, 123, 123, 124, 124, 124, 124};

// solenoid duty para atingir 205 Kpa @ 4000 RPM
/*
unsigned int duty_table0[61] = {80, 79, 78, 76, 75, 75, 75, 75, 75, 75, 75, 75, 74, 74, 74, 73, 73, 73, 72, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 64, 63, 62, 62, 62, 62, 61, 61, 61, 61, 61, 60, 60, 60, 60, 60, 59, 59, 58, 58, 57, 56, 55, 55, 55, 55, 55};
unsigned int duty_table1[61] = {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38};
unsigned int duty_table2[61] = {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 64, 64, 63, 63, 62, 62, 61, 61, 60, 60, 59, 59, 58, 58, 57, 57, 56, 56, 55, 55, 54, 54, 53, 53, 52, 52};
unsigned int duty_table3[61] = {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 63, 63, 63, 62, 62, 62, 61, 61, 61, 60, 60, 60, 59, 59, 59, 58, 58, 58, 57, 57, 57, 56};
unsigned int duty_table4[61] = {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 64, 63, 62, 62, 61, 61, 60, 60, 59, 58, 56, 53, 50, 47, 46, 46, 45, 44, 44, 43, 42, 42, 41, 40, 40, 39, 38};
*/
/*
unsigned int dt_table[][61] = {
  {80, 79, 78, 76, 75, 75, 75, 75, 75, 75, 75, 75, 74, 74, 74, 73, 73, 73, 72, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 64, 63, 62, 62, 62, 62, 61, 61, 61, 61, 61, 60, 60, 60, 60, 60, 59, 59, 58, 58, 57, 56, 55, 55, 55, 55, 55},
  {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38},
  {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 64, 64, 63, 63, 62, 62, 61, 61, 60, 60, 59, 59, 58, 58, 57, 57, 56, 56, 55, 55, 54, 54, 53, 53, 52, 52},
  {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 63, 63, 63, 62, 62, 62, 61, 61, 61, 60, 60, 60, 59, 59, 59, 58, 58, 58, 57, 57, 57, 56},
  {80, 80, 79, 79, 78, 78, 77, 77, 76, 76, 75, 75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 66, 66, 66, 65, 64, 63, 62, 62, 61, 61, 60, 60, 59, 58, 56, 53, 50, 47, 46, 46, 45, 44, 44, 43, 42, 42, 41, 40, 40, 39, 38}
};
unsigned int *duty_table = dt_table[duty_table_selector];
*/
// int error_table[] = {20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -12, -14, -16, -18, -20};

#define NUM_MAPAS 5
//                             750 1000 1250 1500 1750 2000 2250 2500 2750 3000 3250 3500 3750 4000 4250 4500 4750 5000 5250 5500 5750 6000 6250 6500
unsigned int boost_table0[] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
unsigned int boost_table1[] = {175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175};
unsigned int boost_table2[] = {190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190};
unsigned int boost_table3[] = {205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 205, 200, 190, 190, 190, 190};
unsigned int boost_table4[] = {210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 210, 205, 200, 190, 190, 190};
unsigned int boost_table5[] = {215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 215, 210, 205, 200, 190, 190, 190};
unsigned int *mapa[] = {boost_table0, boost_table1, boost_table2, boost_table3, boost_table4, boost_table5};

HardwareTimer *timer;

HardwareSerial debug(PA3, PA2); // RX, TX
Rotary enc(ROT_SW, ROT_CK, ROT_DT);
Fase fase(PHASE_PIN);

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

unsigned int loop_counter = 0;
unsigned int map_time = 0;

void loop()
{
  uint32_t raw_map = analogRead(MAP_PIN);
  // map_value = (raw_map * map_cal) / 1000; // map_cal --> x1000
  map_sum += (raw_map * map_cal) / 1000;
  loop_counter++;
  if ((millis() - map_time) > 5)
  {
    map_value = map_sum / loop_counter;
    map_sum = 0;
    loop_counter = 0;
    map_time = millis();
  }

  switch (overboost_status)
  {
  case 0: // check
    if (map_value > MAP_MAX)
    {
      if (state == PEAK || state == SPOOL)
      {
        overboost_time = millis();
        overboost_status = 1;
      }
    }
    break;
  case 1:
    if ((millis() - overboost_time) > overboost_max_time)
    {
      if (map_value > MAP_MAX)
      {
        // OVERBOOST!
        analogWrite(WG_PIN, 0);
        overboost_count++;
        for (int i = 0; i < 10; i++)
        {
          oled.setTextColor(0, 1);
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.setTextSize(2);
          oled.println("OVERBOOST");
          // oled.setCursor(0, 32);
          oled.print("> ");
          oled.print(MAP_MAX);
          oled.println(" kpa");
          oled.print("PEAK: ");
          oled.println(map_value);
          oled.display();
          delay(100);
          oled.setTextColor(1, 0);
          oled.clearDisplay();
          oled.setCursor(0, 0);
          oled.setTextSize(2);
          oled.println("OVERBOOST");
          // oled.setCursor(0, 32);
          oled.print("> ");
          oled.print(MAP_MAX);
          oled.println(" kpa");
          oled.print("PEAK: ");
          oled.println(map_value);
          oled.display();
          delay(100);
        }
        integral_error = 0;
        i_enabled = false;
        map_sum = 0;
        loop_counter = 0;
        map_time = millis();
      }
      // RESET
      overboost_status = 0;
      overboost_time = 0;
    }
    break;
  }

  if (map_value < MAP_MIN)
  {
    status = 'e';
  }

  digitalWrite(LED_BUILTIN, ledState);

  unsigned int rpm = fase.getRPM();
  unsigned int rpm_index = get_rpm_index(rpm);

  boost_req = mapa[map_num][rpm_index];
  boost_error = boost_req - map_value;

  unsigned int error_index = get_error_index(boost_error);

  if (map_value > peakMap)
  {
    peakMap = map_value;
    peakRPM = rpm;
  }

  if (rpm < 900)
  {
    state = IDLE;
  }

  switch (status)
  {
  default:
  case 'n': // padrão
            // ###### OPERAÇÃO NORMAL #######
    switch (state)
    {

    case IDLE:
      duty = 80;
      if (rpm > 950)
      {
        state = SPOOL;
      }
      break;

    case SPOOL:
      if (boost_error < spool_end_error)
      {
        peakMap = 0;
        peak_state = 0;
        state = PEAK;
        peak_error = boost_error;
        peak_start_time = millis();
      }
      else
      {
        duty = DUTY_MAX;
      }
      break;

    case PEAK:
      // duty = (((duty_table[error_index]*((boost_req*100)/base_kpa))/100) * rpm_duty_mul[rpm_index]) / 100;
      if ((millis() - peak_start_time) > peak_timeout)
      {
        // peak timeout
        state = MESA;
      }
      if (boost_error < peak_error)
      {
        peak_error = boost_error;
      }

      switch (peak_state)
      {
      case 0: // ANTES DO ERRO CHEGAR EM 0 - boost subindo
        peak_duty_mul = peak_spool_mul;
        if (boost_error < 0)
        {
          peak_state = 1;
          pk_last_error = boost_error - 1;
        }
        break;
      case 1: // PEAK MID - boost estabilizando - pico da onda - encontrar max pk
        peak_duty_mul = peak_over_mul;
        if (((boost_error - 2) > peak_error) && ((pk_last_error - (boost_error)) < 0))
        {
          peak_state = 2;
          peak_error = 0;
        }
        else
        {
          pk_last_error = boost_error;
        }
        break;
      case 2: // PEAK END - boost caindo até o setpoint
        peak_duty_mul = peak_end_mul;
        if (boost_error > peak_end_error)
        {
          state = MESA;
          peak_state = 0;
        }
        break;
        break;
      }
      duty = (calc_duty(rpm_index, boost_error, boost_req) * peak_duty_mul) / 100; //(calc_duty(rpm_index, error_index, base_kpa, boost_req) * peak_duty_mul) / 100;
      if (boost_error > cut_error)
      {
        state = CUT;
        cut_time = millis();
      }
      break;

    case MESA:
      i_enabled = true;
      duty = calc_duty(rpm_index, boost_error, boost_req); // calc_duty(rpm_index, error_index, base_kpa, boost_req);
      // duty = ((((duty_table[error_index]*((boost_req*100)/base_kpa))/100) * rpm_duty_mul[rpm_index]) / 100) + p_out + i_out;
      //  se o erro aumentar muito e rapidamente --> CUT
      if (boost_error > cut_error)
      {
        state = CUT;
        cut_time = millis();
      }
      break;

    case CUT:
      duty = DUTY_MIN;
      i_enabled = false;
      // wait and go back to idle
      if ((millis() - cut_time) > 10)
      {
        state = IDLE;
      }
      break;
    }

    // ###### FIM OPERAÇÃO NORMAL #######
    break;
  case 'e': // modo de erro - pressão inválida
    duty = 0;
    i_enabled = false;
    integral_error = 0;
    if (map_value > (MAP_MIN + 10))
    {
      status = 'n';
    }
    if ((millis() - last_led_time) > 100)
    {
      ledState = !ledState;
      last_led_time = millis();
    }
    break;
  case 't': // modo de teste
    static int step = 4;
    if ((millis() - test_time) > test_delay)
    {
      test_time = millis();
      duty = duty + step;
      if (duty > PWM_MAX)
      {
        duty = PWM_MAX;
        step = -4;
      }
      else if (duty < PWM_MIN)
      {
        duty = PWM_MIN;
        step = 4;
      }
    }
    break;
  case 'm': // modo manual
    break;
  }

  if (duty > PWM_MAX)
  {
    duty = PWM_MAX;
  }
  else if (duty < PWM_MIN)
  {
    duty = PWM_MIN;
  }

  analogWrite(WG_PIN, (duty * 255) / 100);

  // DISPLAY
  if ((millis() - last_display_time) > 30)
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
      oled.print(rpm);
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
      oled.print(mapa[map_num][10]);
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

    oled.display();
    last_display_time = millis();
  }
}

void enc_isr()
{
  enc.poll();
}

void sw_isr()
{
  enc.pollSW();
}

void phase_isr()
{
  fase.pulseISR();
}

int b_rate_counter = 0;
void control_isr() // rodando a cada 1 ms
{
  if (b_rate_counter > 19) // 20 ciclos --> 20ms
  {
    int boost_change = boost_error - last_boost_error;
    error_change_rate = (boost_change * 1000) / 20; // boost change rate --> kpa/s
    last_boost_error = boost_error;
    b_rate_counter = 0;
  }
  b_rate_counter++;

  // integral

  if (i_enabled)
  {
    p_out = (pid_kp * boost_error) / 100;
    integral_error += boost_error * 1; // PID_PERIOD = 0.001s = 1 ms;
    if (integral_error > integral_limit)
    {
      integral_error = integral_limit;
    }
    else if (integral_error < (-integral_limit))
    {
      integral_error = -integral_limit;
    }
    i_out = (pid_ki * integral_error) / 100000; // pid_ki -> x100, integral -> x1000 (1 ms) ----> x100000
  }
  else
  {
    p_out = 0;
    integral_error = 0;
    i_out = 0;
  }
}

/*
int calc_duty(int rpm_index, int err_index, int base_kpa, int boost_req)
{
  int boost_ratio100 = (boost_req * 100) / base_kpa;
  int d = (duty_table[err_index] * boost_ratio100 * rpm_duty_mul[rpm_index]) / 10000;
  // duty = ((((duty_table[error_index]*((boost_req*100)/base_kpa))/100) * rpm_duty_mul[rpm_index]) / 100) + p_out + i_out;
  if (i_enabled)
  {
    d = d + p_out + i_out;
  }
  if (d > DUTY_MAX)
  {
    d = DUTY_MAX;
  }
  else if (d < DUTY_MIN)
  {
    d = DUTY_MIN;
  }
  return d;
}
*/

// ((err_base - err_atual) * K) + 66
// duty = (base_duty * boost_req) / base_kpa
int calc_duty(int _rpm_index, int _err, int _boost_req)
{
  int d = 0;

  // cálculo duty base
  int base_x100 = ((_boost_req * 100) / base_kpa) * base_duty; // cálculo do duty base para a pressão requisitada
  if (_err > err_base_start)
  { // antes do setpoint
    d = _err * k_duty_under;
  }
  else if (_err < err_base_end)
  { // após setpoint
    d = _err * k_duty_over;
  }
  d = d + base_x100;

  // compensação por RPM
  d = d * rpm_duty_mul[_rpm_index]; // (x100 * x100 --> x10000)

  // conversão x10000 --> x1
  d = d / 10000;

  // PID
  if (i_enabled)
  {
    d = d + p_out + i_out;
  }

  // limites de duty
  if (d > DUTY_MAX)
  {
    d = DUTY_MAX;
  }
  else if (d < DUTY_MIN)
  {
    d = DUTY_MIN;
  }

  return d;
}

void control_isr2() {
  int abs_pressure = (analogRead(MAP_PIN) * map_cal); // pascal
}