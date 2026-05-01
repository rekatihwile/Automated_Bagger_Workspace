#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

// ===================== DRIVER / MOTOR SETTINGS =====================
#define R_SENSE 0.11f              // Common value on many TMC2209 modules; verify your board
#define DRIVER_ADDRESS 0b00        // Usually 0b00 unless MS1/MS2 address pins changed

// TMC2209 current is configured in mA RMS
// Start conservative. Your motor is rated 2.0 A/phase, but don't start there.
#define RMS_CURRENT_MA 1200

// ===================== PIN ASSIGNMENTS =====================
const uint8_t STEP_PIN   = 6;
const uint8_t DIR_PIN    = 7;
const uint8_t ENABLE_PIN = 8;

// UART pins for TMC2209
const uint8_t SW_RX = 10;   // Arduino receives on this pin
const uint8_t SW_TX = 11;   // Arduino transmits on this pin

// ===================== OBJECTS =====================
SoftwareSerial TMCSerial(SW_RX, SW_TX);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ===================== MOTION SETTINGS =====================
const float MAX_SPEED = 1200.0;   // steps/second
String inputLine = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);   // LOW usually enables TMC2209 modules

  // STEP/DIR stepper settings
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setMinPulseWidth(2);

  // UART to driver
  TMCSerial.begin(115200);

  // ---------------- TMC2209 CONFIG ----------------
  driver.begin();              // UART: initialize driver
  driver.toff(4);              // Enable driver
  driver.blank_time(24);
  driver.rms_current(RMS_CURRENT_MA);   // RMS current in mA
  driver.microsteps(16);       // 1/16 microsteps
  driver.en_spreadCycle(false); // false = StealthChop, true = SpreadCycle
  driver.pdn_disable(true);    // Use UART
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);

  // Optional tuning
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(10);

  Serial.println("Ready.");
  Serial.println("Send one absolute position in steps.");
  Serial.println("Example: 1600");

  // Basic UART sanity check
  Serial.print("DRV_STATUS: 0x");
  Serial.println(driver.DRV_STATUS(), HEX);
}

void loop() {
  readSerialCommand();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        processCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();

  if (!isValidInteger(cmd)) {
    Serial.println("ERR: expected one integer like 1600");
    return;
  }

  long target = cmd.toInt();

  Serial.print("Moving to: ");
  Serial.println(target);

  stepper.moveTo(target);
  stepper.runToPosition();

  stepper.setCurrentPosition(target);

  Serial.println("DONE");
}

bool isValidInteger(const String &s) {
  if (s.length() == 0) return false;

  int start = 0;
  if (s[0] == '-' || s[0] == '+') {
    if (s.length() == 1) return false;
    start = 1;
  }

  for (int i = start; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }

  return true;
}