#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// ============================================================
// COOPER WIRING / TMC2209 UART
// ============================================================

#define R_SENSE 0.11f

// UART communication pins from Cooper's setup
#define SW_RX 9
#define SW_TX 8
SoftwareSerial tmcSerial(SW_RX, SW_TX);

// Motor 1 pins
#define STEP1 2
#define DIR1  3
#define EN1   4

// Motor 2 pins
#define STEP2 5
#define DIR2  6
#define EN2   7

// TMC2209 UART addresses
// Cooper's code used address 0 for driver1 and 1 for driver2.
TMC2209Stepper driver1(&tmcSerial, R_SENSE, 0);
TMC2209Stepper driver2(&tmcSerial, R_SENSE, 1);

// AccelStepper objects
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);

// ============================================================
// END EFFECTOR / GRIPPER SERVO
// ============================================================

// IMPORTANT:
// Cooper uses pin 9 for TMC UART RX.
// So the servo signal must move off D9.
// Plug servo signal into Arduino D10.
#define SERVO_PIN 10

#define MIN_GRIPPER_ANGLE 5
#define MAX_GRIPPER_ANGLE 75

Servo gripper;

// ============================================================
// SETTINGS
// ============================================================

#define RMS_CURRENT_MA 1400

// Cooper used these values
#define MAX_SPEED 90000
#define ACCELERATION 30000

String inputLine = "";

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  tmcSerial.begin(115200);

  // Enable pins
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  // TMC enable is active LOW:
  // LOW = enabled / holding
  // HIGH = disabled / limp
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);

  // Gripper servo
  gripper.attach(SERVO_PIN);
  gripper.write(MIN_GRIPPER_ANGLE);

  // TMC2209 setup
  setupDriver(driver1);
  setupDriver(driver2);

  // Motion settings
  motor1.setMaxSpeed(MAX_SPEED);
  motor1.setAcceleration(ACCELERATION);
  motor1.setMinPulseWidth(2);

  motor2.setMaxSpeed(MAX_SPEED);
  motor2.setAcceleration(ACCELERATION);
  motor2.setMinPulseWidth(2);

  Serial.println("READY");
  Serial.println("Cooper wiring controller ready.");
  Serial.println("Commands:");
  Serial.println("M1 <pos>      move motor 1 to absolute position");
  Serial.println("M2 <pos>      move motor 2 to absolute position");
  Serial.println("B <pos>       move both motors to same absolute position");
  Serial.println("J1 <steps>    jog motor 1 relative");
  Serial.println("J2 <steps>    jog motor 2 relative");
  Serial.println("J <steps>     jog both motors relative");
  Serial.println("A <angle>     gripper angle 5-75");
  Serial.println("EN 1          enable / hold motors");
  Serial.println("EN 0          disable motors");
  Serial.println("H             force hold motors");
  Serial.println("Z             zero both motor positions");
  Serial.println("POS           print motor positions");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // Keep motors running toward targets
  motor1.run();
  motor2.run();

  readSerialCommands();
}

// ============================================================
// DRIVER SETUP
// ============================================================

void setupDriver(TMC2209Stepper &driver) {
  driver.begin();

  // Enable output stage
  driver.toff(4);
  driver.blank_time(24);

  // Current
  driver.rms_current(RMS_CURRENT_MA);

  // Cooper used 4 microsteps
  driver.microsteps(4);

  // SpreadCycle gives stronger torque/holding than StealthChop
  driver.en_spreadCycle(true);

  // UART mode
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);

  // Keep hold current high so vertical axis does not drop after motion
  driver.ihold(31);
  driver.irun(31);
  driver.iholddelay(0);
  driver.TPOWERDOWN(0);
}

void forceHold() {
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);

  driver1.ihold(31);
  driver1.irun(31);
  driver1.TPOWERDOWN(0);

  driver2.ihold(31);
  driver2.irun(31);
  driver2.TPOWERDOWN(0);
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      inputLine.trim();

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

  if (cmd.startsWith("M1")) {
    long pos = cmd.substring(2).toInt();
    motor1.moveTo(pos);
    forceHold();

    Serial.print("OK M1 ");
    Serial.println(pos);
  }

  else if (cmd.startsWith("M2")) {
    long pos = cmd.substring(2).toInt();
    motor2.moveTo(pos);
    forceHold();

    Serial.print("OK M2 ");
    Serial.println(pos);
  }

  else if (cmd.startsWith("B")) {
    long pos = cmd.substring(1).toInt();
    motor1.moveTo(pos);
    motor2.moveTo(pos);
    forceHold();

    Serial.print("OK BOTH ");
    Serial.println(pos);
  }

  else if (cmd.startsWith("J1")) {
    long delta = cmd.substring(2).toInt();
    motor1.moveTo(motor1.currentPosition() + delta);
    forceHold();

    Serial.print("OK J1 ");
    Serial.println(delta);
  }

  else if (cmd.startsWith("J2")) {
    long delta = cmd.substring(2).toInt();
    motor2.moveTo(motor2.currentPosition() + delta);
    forceHold();

    Serial.print("OK J2 ");
    Serial.println(delta);
  }

  else if (cmd.startsWith("J")) {
    long delta = cmd.substring(1).toInt();
    motor1.moveTo(motor1.currentPosition() + delta);
    motor2.moveTo(motor2.currentPosition() + delta);
    forceHold();

    Serial.print("OK J ");
    Serial.println(delta);
  }

  else if (cmd.startsWith("A")) {
    int angle = cmd.substring(1).toInt();

    if (angle < MIN_GRIPPER_ANGLE || angle > MAX_GRIPPER_ANGLE) {
      Serial.println("ERR gripper_angle_out_of_range");
      return;
    }

    gripper.write(angle);
    forceHold();

    Serial.print("OK gripper ");
    Serial.println(angle);
  }

  else if (cmd.startsWith("EN")) {
    int enabled = cmd.substring(2).toInt();

    if (enabled == 1) {
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      forceHold();
      Serial.println("OK motors_enabled_holding");
    } else {
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      Serial.println("OK motors_disabled");
    }
  }

  else if (cmd == "H") {
    forceHold();
    Serial.println("OK holding");
  }

  else if (cmd == "Z") {
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    motor1.moveTo(0);
    motor2.moveTo(0);
    forceHold();
    Serial.println("OK zeroed");
  }

  else if (cmd == "POS") {
    Serial.print("P1 ");
    Serial.println(motor1.currentPosition());

    Serial.print("P2 ");
    Serial.println(motor2.currentPosition());
  }

  else if (cmd == "C") {
    setupDriver(driver1);
    setupDriver(driver2);
    forceHold();
    Serial.println("OK drivers_configured");
  }

  else if (cmd == "STOP") {
    motor1.stop();
    motor2.stop();
    forceHold();
    Serial.println("OK stopping");
  }

  else {
    Serial.print("ERR unknown_command ");
    Serial.println(cmd);
  }
}