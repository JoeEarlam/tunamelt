#define _TASK_MICRO_RES
#include <TaskScheduler.h>  //https://github.com/arkhipenko/TaskScheduler
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>  //https://github.com/adafruit/Adafruit_LIS331
#include <Adafruit_Sensor.h>    //H3LIS331 dependency
#include <PIO_DShot.h>          //https://github.com/bastian2001/pico-bidir-dshot
#include <Adafruit_NeoPixel.h>  //https://github.com/adafruit/Adafruit_NeoPixel
#include "melty.h"
#include "src/ibus.h"

Adafruit_NeoPixel strip(1, PIN_RGB_DAT, NEO_GRB + NEO_KHZ800);
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();
FlySkyIBus iBus;

DShotX4 *esc0;
DShotX4 *esc1;

//core0
Radio_t parseChannels(uint16_t (&chanData)[IBUS_CHANNELS]);
void readRadioCB();
void readAccelCB();
void mainTaskCB();
void rgbCB();

Scheduler ts0;

Task readRadio(0, TASK_FOREVER, &readRadioCB, &ts0);
Task readAccel(1000, TASK_FOREVER, &readAccelCB, &ts0);
Task mainTask(0, TASK_ONCE, &mainTaskCB, &ts0);
Task rgb(1000, TASK_FOREVER, &rgbCB, &ts0);

//core1
void meltyTaskCB();
void headingCB();
void translateCB();
void writePWMCB();

Scheduler ts1;

Task meltyTask(0, TASK_FOREVER, &meltyTaskCB, &ts1);
Task heading(0, 2, &headingCB, &ts1);
Task translate(0, TRANSLATE_STEPS, &translateCB, &ts1);
Task writePWM(200, TASK_FOREVER, &writePWMCB, &ts1);

Melty_t melty;
DriveMode_t driveMode;
RadioState_t radioState;
BattState_t battState;
Radio_t rxFrame;

void setup() {
  rp2040.idleOtherCore();

  pinMode(PIN_HEADING, OUTPUT);
  analogWriteFreq(10000); //default frequency too slow, causes dithered heading LED
  analogWrite(PIN_HEADING, 32);

  Serial.begin(115200);  //debug serial

  setupIbus();
  setupAccel();
  setupDshot();
  setupRGB();

  ts0.enableAll();
  writePWM.enable();  // enable PWM task but leave others disabled for now

  rp2040.resumeOtherCore();
  rp2040.wdt_begin(1000);
  analogWrite(PIN_HEADING, 0);
}

void setup1() {
  delay(1);
}

void loop() {
  ts0.execute();
}

void loop1() {
  ts1.execute();
}

void readRadioCB() {
  static uint32_t lastRadioTime;
  uint16_t chanData[IBUS_CHANNELS];

  iBus.loop();

  if (iBus.readChannels(chanData)) {
    lastRadioTime = millis();
    radioState = RADIO_OK;
    rxFrame = parseChannels(chanData);
    mainTask.restart();
  }

  if (millis() > (lastRadioTime + 100)) {
    lastRadioTime = millis();
    radioState = RADIO_DC;
    Radio_t emptyFrame;
    rxFrame = emptyFrame;
    mainTask.restart();
  }
}

/*
  Logic and controls
*/
void mainTaskCB() {
  float accelTrim = float(rxFrame.ch5) / 100;
  melty.accelRadius = ACCEL_DIST + accelTrim;
  melty.battVolts = readBattVolts();
  battState = getBattState();

  if (rxFrame.ch3 == 0) driveMode = TANK;
  else {
    if (rxFrame.ch6 > 0) driveMode = MELTY_R;
    else driveMode = MELTY_F;
  }

  switch (driveMode) {
    case TANK:
      meltyTask.disable();
      melty.spinDuty = 0;
      tankDrive(rxFrame.steering, rxFrame.throttle);
      break;

    case MELTY_F:
      if (rxFrame.ch3 == 1) melty.spinDuty = SPEED_LO;
      if (rxFrame.ch3 == 2) melty.spinDuty = SPEED_HI;
      break;

    case MELTY_R:
      if (rxFrame.ch3 == 1) melty.spinDuty = -SPEED_LO;
      if (rxFrame.ch3 == 2) melty.spinDuty = -SPEED_HI;
      break;

    default:

      break;
  }

  if (driveMode != TANK) {
    if (melty.rpm < 60) {
      melty.pwm0 = melty.spinDuty;
      melty.pwm1 = melty.spinDuty;
    } else {
      if (!meltyTask.isEnabled()) {
        meltyTask.restart();
        meltyTask.setInterval(1000000 / (melty.rpm / 60));
      }
    }
  }
}

/*
  Main melty task
*/
void meltyTaskCB() {
  int steerVal = rxFrame.steering * 7;
  if (driveMode == MELTY_R) steerVal = -steerVal;  //steering inverted when spinning backwards

  int period = 1000000 / (melty.rpm / 60);
  period = period + steerVal;

  heading.setInterval(degToMicros(period, HEADING_ANGLE));
  heading.restartDelayed(degToMicros(period, 90 - (HEADING_ANGLE / 2)));
  meltyTask.setIntervalNodelay(period, TASK_INTERVAL_RECALC);
  translate.setIntervalNodelay(period / TRANSLATE_STEPS, TASK_INTERVAL_RECALC);
  translate.restart();
}

void translateCB() {
  int m0, m1;

  int i = translate.getRunCounter() - 1;
  float angle = ((2 * PI) / TRANSLATE_STEPS) * i;

  int dutyDelta = 255 - abs(melty.spinDuty);
  float magnitude = float(rxFrame.throttle) / 255;
  float cosineval = cos(angle) * dutyDelta * magnitude;

  m0 = abs(melty.spinDuty) + cosineval;
  m1 = abs(melty.spinDuty) - cosineval;

  //invert values and send to opposite motors in reverse to preserve translation direction
  if (driveMode == MELTY_R) {
    melty.pwm0 = -constrain(m1, 1, 255);
    melty.pwm1 = -constrain(m0, 1, 255);
  } else {
    melty.pwm0 = constrain(m0, 1, 255);
    melty.pwm1 = constrain(m1, 1, 255);
  }
}

void headingCB() {
  if (heading.isFirstIteration()) {
    analogWrite(PIN_HEADING, LED_BRIGHTNESS);
  } else analogWrite(PIN_HEADING, 0);
}

void readAccelCB() {
  sensors_event_t event;
  lis.getEvent(&event);  //1.5ms

  float y = abs(event.acceleration.y / SENSORS_GRAVITY_STANDARD);
  float x = abs(event.acceleration.x / SENSORS_GRAVITY_STANDARD);

  melty.accel = sqrt((y * y) + (x * x));

  if (melty.accel < 5) {
    melty.accel = 0;
    melty.rpm = 0;
  } else {
    melty.rpm = sqrt(melty.accel / (melty.accelRadius * 1.118e-6));
  }
}

void writePWMCB() {
  uint16_t throttles0[4] = { 0, 0, 0, 0 };
  uint16_t throttles1[4] = { 0, 0, 0, 0 };

  if (radioState == RADIO_OK) {
    throttles0[0] = pwmToDshot(melty.pwm0);
    throttles1[0] = pwmToDshot(melty.pwm1);
  }

  esc0->sendThrottles(throttles0);
  esc1->sendThrottles(throttles1);
  rp2040.wdt_reset();
}

void rgbCB() {
  if (battState == BATT_LOW) strip.fill(strip.Color(255, 0, 0));
  else if (radioState == RADIO_DC) strip.fill(strip.Color(0, 0, 255));
  else rgbRainbow();

  strip.show();
}

Radio_t parseChannels(uint16_t (&chanData)[IBUS_CHANNELS]) {
  Radio_t thisFrame;

  thisFrame.steering = map(chanData[0], 1000, 2000, -255, 255);
  thisFrame.throttle = map(chanData[1], 1000, 2000, -255, 255);
  thisFrame.ch3 = map(chanData[2], 1000, 2000, 0, 2);  //3 pos switch
  thisFrame.ch4 = map(chanData[3], 1000, 2000, -255, 255);
  thisFrame.ch5 = map(chanData[4], 1000, 2000, -255, 255);  //potentiometer or trim switch
  thisFrame.ch6 = map(chanData[5], 1000, 2000, -255, 255);
  thisFrame.ch7 = map(chanData[6], 1000, 2000, -255, 255);  //2 pos switch
  thisFrame.ch8 = map(chanData[7], 1000, 2000, -255, 255);
  thisFrame.ch9 = map(chanData[8], 1000, 2000, -255, 255);
  thisFrame.ch10 = map(chanData[9], 1000, 2000, -255, 255);
  thisFrame.ch11 = map(chanData[10], 1000, 2000, -255, 255);
  thisFrame.ch12 = map(chanData[11], 1000, 2000, -255, 255);
  thisFrame.ch13 = map(chanData[12], 1000, 2000, -255, 255);
  thisFrame.ch14 = chanData[13];

  thisFrame.steering = constrain(thisFrame.steering, -255, 255);
  thisFrame.throttle = constrain(thisFrame.throttle, -255, 255);

  return thisFrame;
}

BattState_t getBattState() {
  int lowBattRst = LOW_BATT_MVOLTS + 400;
  static bool lowBattEvent;

  if (melty.battVolts < LOW_BATT_MVOLTS) lowBattEvent = true;
  if (melty.battVolts > lowBattRst) lowBattEvent = false;

  if (lowBattEvent) return BATT_LOW;
  else return BATT_OK;
}

int readBattVolts() {
  int adc = analogRead(PIN_VBATT);
  const int vRef = 3300;
  const uint16_t fsr = 1023;

  adc = adc * (vRef / fsr);

  adc = adc * (1 + (VBATT_R1 / VBATT_R2));
  return adc;
}

void rgbRainbow() {
  static uint16_t hue;

  //Slow rainbow if not spinning, else draw persistence of vision rainbow
  if (driveMode == TANK) {
    static uint32_t lastMillis;
    uint32_t currentMillis = millis();

    hue = hue + (currentMillis - lastMillis) * 16;
    lastMillis = currentMillis;
  } else {
    int iteration = translate.getIterations() - 1;
    if (iteration >= TRANSLATE_STEPS) iteration = 0;
    hue = (65535 / TRANSLATE_STEPS) * iteration;
  }
  strip.rainbow(hue);
}

void tankDrive(int s, int t) {
  s = s / 4;
  melty.pwm0 = constrain(t + s, -255, 255);
  melty.pwm1 = -constrain(t - s, -255, 255);
}

/*
  Library speed range is 1-1000 fwd, 1001-2000 reverse, 0 = stop
*/
int pwmToDshot(int pwm) {
  int dshot = 0;
  pwm = constrain(pwm, -255, 255);

  int minpwm;
  if (driveMode == TANK) minpwm = 10;
  else minpwm = 2;

  //avoid sending full stop signal
  if ((pwm > 0) && (pwm < minpwm)) return 1;
  if ((pwm < 0) && (pwm > -minpwm)) return 1001;

  if (pwm > 0) {
    dshot = map(pwm, minpwm, 255, 1, 1000);
    dshot = constrain(dshot, 1, 1000);
  }
  if (pwm < 0) {
    dshot = map(abs(pwm), minpwm, 255, 1001, 2000);
    dshot = constrain(dshot, 1001, 2000);
  }

  return dshot;
}

int degToMicros(int interval, int deg) {
  interval = (interval / 360) * deg;
  return interval;
}

void setupIbus() {
  Serial1.setFIFOSize(64);  //default 32 is too small
  Serial1.setTX(-1);        //no tx pin
  Serial1.begin(115200);
  iBus.begin(Serial1);
}

void setupDshot() {
  esc0 = new DShotX4(PIN_ESC0, 1);
  esc1 = new DShotX4(PIN_ESC1, 1);
  //arm ESCs
  uint16_t throttles[4] = { 0, 0, 0, 0 };
  esc0->sendThrottles(throttles);
  esc1->sendThrottles(throttles);
  delay(500);
}

void setupRGB() {
  pinMode(PIN_RGB_EN, OUTPUT);
  digitalWrite(PIN_RGB_EN, HIGH);
  strip.begin();
  strip.show();
  strip.setBrightness(255);
}

void setupAccel() {
  if (!lis.begin_I2C(0x19)) {
    Serial.println("H3LIS331 not found!");
    delay(500);
    rp2040.reboot();
  } else Serial.println("H3LIS331 found!");

  lis.setRange(H3LIS331_RANGE_400_G);
  lis.setDataRate(LIS331_DATARATE_100_HZ);
  lis.setLPFCutoff(LIS331_LPF_37_HZ);
}
