#ifndef MELTY_H
#define MELTY_H

#include <Arduino.h>

#define SPEED_LO 100
#define SPEED_HI 140
#define ACCEL_DIST 19
#define ESC_COUNT 2
#define HEADING_ANGLE 45
#define LED_BRIGHTNESS 255
#define TRANSLATE_STEPS 32
#define VBATT_R1 47000
#define VBATT_R2 4700
#define LOW_BATT_MVOLTS 6000

#define PIN_HEADING 3
#define PIN_VBATT 29
#define PIN_ESC0 0
#define PIN_ESC1 2
#define PIN_RGB_EN 11
#define PIN_RGB_DAT 12

struct Radio_t {
  int16_t steering = 0;
  int16_t throttle = 0;
  int16_t ch3 = 0;
  int16_t ch4 = 0;
  int16_t ch5 = 0;
  int16_t ch6 = 0;
  int16_t ch7 = 0;
  int16_t ch8 = 0;
  int16_t ch9 = 0;
  int16_t ch10 = 0;
  int16_t ch11 = 0;
  int16_t ch12 = 0;
  int16_t ch13 = 0;
  int16_t ch14 = 0;
};

enum DriveMode_t {
  TANK,
  MELTY_F,
  MELTY_R,
};

struct Melty_t {
  int spinDuty;
  int pwm0;
  int pwm1;
  int battVolts;
  float accel;
  float rpm;
  float accelRadius;
};

enum RadioState_t {
  RADIO_DC,
  RADIO_OK,
};

enum BattState_t {
  BATT_OK,
  BATT_LOW,
};

#endif