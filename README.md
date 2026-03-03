# Advanced-Speedometer-Calibration-Device
ASCD Project (Automatic Speed Calculation Device) ASCD is an embedded system project designed to calculate and display speed based on distance and time measurements. The system was developed using microcontrollers and sensor integration, with a structured design approach based on Finite State Machine (FSM). I
#include <Arduino.h>
#include <Wire.h>
#include <Ds1302.h>
#include "VLXSensor.h"
#include "lcdDisplay.h"

// ====== DS1302 PINS ======
#define RST_PIN 4
#define CLK_PIN 2
#define DAT_PIN 3

// Create Ds1302 object
Ds1302 rtc(RST_PIN, CLK_PIN, DAT_PIN);

// ====== BUZZER ======
const int buzzerPin = 5;         // Buzzer pin
const uint8_t BUZZER_ON  = LOW; // adjust if active LOW
const uint8_t BUZZER_OFF = HIGH;

// ====== TIME & DATE ======
int Hours, minutes, seconds, lastSecond, lastMinute, lastHour;
uint8_t Day, Month, Year;
uint8_t lastDay, lastMonth, lastYear;

// ====== SPEEDOMETER CONFIG ======
const float SENSOR_DISTANCE_CM = 18.0f;  // distance between point A and B
int   thresholdDistance = 25;            // detection threshold in cm
float speedKmh = 0.0f;                   // last measured speed (km/h), kept until next valid measurement

// For LCD change detection
int pointA = 0;
int pointB = 0;
int lastPointA = -1;
int lastPointB = -1;
int lastSpeed10 = -1;  // speedKmh * 10 as int for change detection

// ====== SPEED MEASUREMENT STATE MACHINE ======
enum MeasureState {
  WAITING_FOR_OBJECT,
  TIMING,
  BREAK_AFTER_MEASUREMENT
};

MeasureState measureState = WAITING_FOR_OBJECT;
char startSensor = 0;                  // 'A' or 'B'
unsigned long startTimeMs = 0;

const unsigned long MEASURE_TIMEOUT_MS = 5000; // cancel if second sensor not hit in 5s
const unsigned long BREAK_TIME_MS      = 3000; // 3s break between VALID measurements

unsigned long breakStartMs = 0;

// Break countdown display
int  breakSeconds      = 0;
int  lastBreakSeconds  = -1;

// Edge detection for sensors
bool prevADetected = false;
bool prevBDetected = false;

void setup() {
  Serial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, BUZZER_OFF);

  initVLXSensors(); // Initialize VLX sensors

  rtc.init();

  // If oscillator was halted, restart it (according to your lib)
  if (rtc.isHalted()) {
    Serial.println("Oscillator was halted, starting it...");
    rtc.halt(); // your library’s way to clear halt
  }

  lcdSetup();
}

void loop() {
  // ====== READ VLX SENSORS ======
  readVLXDistances(); // updates Distances.distance1 and distance2 in mm

  // mm -> cm
  pointA = Distances.distance1 / 10.0;
  pointB = Distances.distance2 / 10.0;

  bool aDetected = (pointA < thresholdDistance);
  bool bDetected = (pointB < thresholdDistance);

  // RISING EDGES
  bool aRising = aDetected && !prevADetected;
  bool bRising = bDetected && !prevBDetected;

  // ====== READ RTC TIME ======
  Ds1302::DateTime now;
  rtc.getDateTime(&now);

  Hours   = now.hour;
  minutes = now.minute;
  seconds = now.second;
  Day     = now.day;
  Month   = now.month;
  Year    = now.year;

  unsigned long currentMs = millis();

  // ====== STATE MACHINE ======
  switch (measureState) {
    case WAITING_FOR_OBJECT:
      digitalWrite(buzzerPin, BUZZER_OFF);
      // speedKmh is NOT reset here → last result stays on LCD

      // If both sensors have a rising edge in the same loop → ignore (simultaneous)
      if (aRising && bRising) {
        Serial.println("Simultaneous trigger A+B: ignoring.");
        // Stay waiting
      }
      else if (aRising) {
        // A first → timing A -> B
        measureState = TIMING;
        startSensor = 'A';
        startTimeMs = currentMs;
        digitalWrite(buzzerPin, BUZZER_ON);
        Serial.println("Start timing: A first");
      }
      else if (bRising) {
        // B first → timing B -> A
        measureState = TIMING;
        startSensor = 'B';
        startTimeMs = currentMs;
        digitalWrite(buzzerPin, BUZZER_ON);
        Serial.println("Start timing: B first");
      }
      break;

    case TIMING: {
      // timeout: no second sensor hit → no break, just reset to WAITING
      if (currentMs - startTimeMs > MEASURE_TIMEOUT_MS) {
        Serial.println("Measurement timeout, resetting (no cooldown).");
        measureState = WAITING_FOR_OBJECT;
        digitalWrite(buzzerPin, BUZZER_OFF);
        // DO NOT change speedKmh, last valid result stays
        break;
      }

      if (startSensor == 'A') {
        // Wait for B rising edge
        if (bRising) {
          unsigned long elapsedMs = currentMs - startTimeMs;
          float elapsedSec = elapsedMs / 1000.0f;

          if (elapsedSec > 0.0f) {
            float distance_m = SENSOR_DISTANCE_CM / 100.0f; // cm -> m
            float speedMs = distance_m / elapsedSec;        // m/s
            speedKmh = speedMs * 3.6f;                      // km/h
          }

          Serial.print("A -> B: elapsed [ms] = ");
          Serial.print(elapsedMs);
          Serial.print(", speed [km/h] = ");
          Serial.println(speedKmh, 2);

          digitalWrite(buzzerPin, BUZZER_OFF);
          // VALID measurement finished → start cooldown
          measureState  = BREAK_AFTER_MEASUREMENT;
          breakStartMs  = currentMs;
        }
      } else if (startSensor == 'B') {
        // Wait for A rising edge
        if (aRising) {
          unsigned long elapsedMs = currentMs - startTimeMs;
          float elapsedSec = elapsedMs / 1000.0f;

          if (elapsedSec > 0.0f) {
            float distance_m = SENSOR_DISTANCE_CM / 100.0f; // cm -> m
            float speedMs = distance_m / elapsedSec;        // m/s
            speedKmh = speedMs * 3.6f;                      // km/h
          }

          Serial.print("B -> A: elapsed [ms] = ");
          Serial.print(elapsedMs);
          Serial.print(", speed [km/h] = ");
          Serial.println(speedKmh, 2);

          digitalWrite(buzzerPin, BUZZER_OFF);
          // VALID measurement finished → start cooldown
          measureState  = BREAK_AFTER_MEASUREMENT;
          breakStartMs  = currentMs;
        }
      }
    } break;

    case BREAK_AFTER_MEASUREMENT:
      // 3s cooldown + wait until both sensors are clear before arming again
      if ((currentMs - breakStartMs) >= BREAK_TIME_MS &&
          !aDetected && !bDetected) {
        Serial.println("Break finished, ready for next measurement.");
        measureState = WAITING_FOR_OBJECT;
      }
      digitalWrite(buzzerPin, BUZZER_OFF);
      break;
  }

  // ====== BREAK COUNTDOWN COMPUTATION (only after VALID measurement) ======
  if (measureState == BREAK_AFTER_MEASUREMENT) {
    unsigned long elapsedBreak = currentMs - breakStartMs;
    if (elapsedBreak >= BREAK_TIME_MS) {
      breakSeconds = 0;
    } else {
      // ceil((BREAK_TIME_MS - elapsedBreak) / 1000.0)
      unsigned long remainingMs = BREAK_TIME_MS - elapsedBreak;
      breakSeconds = (int)((remainingMs + 999) / 1000);
    }
  } else {
    breakSeconds = 0;
  }

  // store previous detection states for next loop (edge detection)
  prevADetected = aDetected;
  prevBDetected = bDetected;

  // ====== LCD UPDATE (only when something changes) ======
  int currentSpeed10 = (int)(speedKmh * 10.0f); // speedKmh * 10 for comparison (one decimal)

  bool changed =
    (Hours   != lastHour)   ||
    (minutes != lastMinute) ||
    (seconds != lastSecond) ||
    (Day     != lastDay)    ||
    (Month   != lastMonth)  ||
    (Year    != lastYear)   ||
    (pointA  != lastPointA) ||
    (pointB  != lastPointB) ||
    (currentSpeed10 != lastSpeed10) ||
    (breakSeconds != lastBreakSeconds);

  if (changed) {
    lastHour         = Hours;
    lastMinute       = minutes;
    lastSecond       = seconds;
    lastDay          = Day;
    lastMonth        = Month;
    lastYear         = Year;
    lastPointA       = pointA;
    lastPointB       = pointB;
    lastSpeed10      = currentSpeed10;
    lastBreakSeconds = breakSeconds;

    // --- Distances A/B (cm) ---
    lcd.setCursor(3, 0);
    lcd.print("   ");
    lcd.setCursor(3, 0);
    lcd.print(pointA);

    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(13, 0);
    lcd.print(pointB);

    // --- Date (DD/MM/20YY) ---
    lcd.setCursor(6, 1);
    if (Day < 10) lcd.print('0');
    lcd.print(Day);

    lcd.setCursor(9, 1);
    if (Month < 10) lcd.print('0');
    lcd.print(Month);

    lcd.setCursor(13, 1);
    lcd.print(20);   // assuming 20xx
    if (Year < 10) lcd.print('0');
    lcd.print(Year);

    // --- Time (HH:MM:SS) ---
    lcd.setCursor(5, 2);
    if (Hours < 10) lcd.print('0');
    lcd.print(Hours);

    lcd.setCursor(8, 2);
    if (minutes < 10) lcd.print('0');
    lcd.print(minutes);

    lcd.setCursor(11, 2);
    if (seconds < 10) lcd.print('0');
    lcd.print(seconds);

    // --- Speed (km/h) + Break countdown ---
    lcd.setCursor(7, 3);
    lcd.print(speedKmh, 1);   // 1 decimal place

    if (breakSeconds > 0) {
      // show countdown on same line: e.g. " T:3s"
      lcd.setCursor(15, 2);
      lcd.print("T:");
      lcd.print(breakSeconds);
      lcd.print("s ");
    }
  }

  // Very small delay to avoid hammering I2C but still react fast
  delay(5);
}
