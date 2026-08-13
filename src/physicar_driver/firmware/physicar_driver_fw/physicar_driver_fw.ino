// AMET 2026 practice platform - motor driver firmware
// Pin mapping confirmed by physical test on 2026-08-13: pin 9 = steering servo, pin 10 = ESC.
//
// Protocol: RPi sends one line per update: "S<steer_us>,T<esc_us>\n"
// e.g. "S1500,T1600\n" (both fields required every line).
// Any line that fails to parse is ignored (no state change, no ack).
//
// Safety:
// - All values clamped to a conservative pulse-width range below.
// - Watchdog: if no valid line received for WATCHDOG_MS, force neutral.
// - Boots holding neutral for BOOT_HOLD_MS before accepting commands.

#include <Servo.h>

Servo steering;  // pin 9
Servo esc;       // pin 10

const int NEUTRAL_US = 1500;

// Conservative pulse-width clamps. Only 1750us has been confirmed to move the
// ESC so far (2026-08-13 bench test); widen these only after further bench
// testing with wheels off the ground.
const int STEER_MIN_US = 1300;
const int STEER_MAX_US = 1700;
const int ESC_MIN_US = 1350;
const int ESC_MAX_US = 1750;

const unsigned long WATCHDOG_MS = 1000;   // matches contest driver's 1s command timeout
const unsigned long BOOT_HOLD_MS = 2000;  // hold neutral at boot so the ESC can arm

unsigned long lastCommandMillis = 0;
bool watchdogTripped = false;

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void applyNeutral() {
  steering.writeMicroseconds(NEUTRAL_US);
  esc.writeMicroseconds(NEUTRAL_US);
}

void setup() {
  Serial.begin(115200);
  steering.attach(9);
  esc.attach(10);
  applyNeutral();
  delay(BOOT_HOLD_MS);
  lastCommandMillis = millis();
  Serial.println("READY physicar_driver_fw");
}

// Parses "S<int>,T<int>" from line. Returns true on success.
bool parseLine(const String &line, int &steerUs, int &escUs) {
  int sIdx = line.indexOf('S');
  int cIdx = line.indexOf(',');
  int tIdx = line.indexOf('T');
  if (sIdx != 0 || cIdx < 0 || tIdx < 0 || tIdx < cIdx) return false;

  String sPart = line.substring(sIdx + 1, cIdx);
  String tPart = line.substring(tIdx + 1);
  sPart.trim();
  tPart.trim();
  if (sPart.length() == 0 || tPart.length() == 0) return false;

  steerUs = sPart.toInt();
  escUs = tPart.toInt();
  return true;
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int steerUs, escUs;
    if (parseLine(line, steerUs, escUs)) {
      steerUs = clampInt(steerUs, STEER_MIN_US, STEER_MAX_US);
      escUs = clampInt(escUs, ESC_MIN_US, ESC_MAX_US);
      steering.writeMicroseconds(steerUs);
      esc.writeMicroseconds(escUs);
      lastCommandMillis = millis();
      if (watchdogTripped) {
        watchdogTripped = false;
        Serial.println("CMD_OK resumed");
      }
      Serial.print("CMD_OK S=");
      Serial.print(steerUs);
      Serial.print(" T=");
      Serial.println(escUs);
    }
  }

  if (millis() - lastCommandMillis > WATCHDOG_MS) {
    applyNeutral();
    if (!watchdogTripped) {
      watchdogTripped = true;
      Serial.println("WATCHDOG neutral");
    }
  }
}
