#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include "Config.h"
#include "Flash.h"
#include "Rotary.h"
#include "Fase.h"
#include "Boost.h"
#include "UI.h"

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

// DISPLAY
bool ledState = false;
uint32_t last_led_time = 0;

TwoWire Wire3(I2C3_SDA, I2C3_SCL);
SPIClass spi1(FLASH_MOSI, FLASH_MISO, FLASH_SCK);
Flash flash(spi1, FLASH_CS);

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

HardwareTimer *timer;

HardwareSerial debug(PA3, PA2); // RX, TX
Rotary enc(ROT_SW, ROT_CK, ROT_DT);
Fase fase(PHASE_PIN);
Boost boost(PID_FREQ, 10);
UI ui(oled, enc, boost, flash);

void enc_isr();
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
  attachInterrupt(digitalPinToInterrupt(ROT_DT), enc_isr, CHANGE);

  attachInterrupt(digitalPinToInterrupt(PHASE_PIN), phase_isr, RISING);
}

int raw_map = 0;
int mbar_map = 0;
int rpm = 0;
int raw_thr = 0;
int thr = 0;

int raw_iat = 0;
int v_iat = 0;

void loop()
{
  enc.updateButton();
  digitalWrite(LED_BUILTIN, ledState);

  ui.update(!digitalRead(BT_PIN), rpm, raw_map, mbar_map, raw_thr, thr);
}

// INTERRUPTS

void control_isr() { // rodando a cada 1 ms
  raw_map = analogRead(MAP_PIN);
  mbar_map = (raw_map * Cfg::map_cal) / 100; // mbar

  raw_thr = analogRead(THROTTLE_PIN);
  thr = (raw_thr * 100) / Cfg::thr_cal; // % pedal acelerador

  //raw_iat = analogRead(IAT_PIN);
  //v_iat = (raw_iat * 3232) / 1000; // converter para mV

  rpm = fase.getRPM();
  //int rpm_index = get_rpm_index(rpm);

  boost.update_boost(mbar_map);
  boost.loop(rpm, thr);
  analogWrite(WG_PIN, (boost.get_data(Boost::DUTY) * 255) / 1000);
}

void phase_isr()
{
  fase.pulseISR();
}

void enc_isr()
{
  enc.poll();
}
