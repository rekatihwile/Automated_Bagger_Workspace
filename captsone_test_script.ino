#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

#define R_SENSE 0.11f

// UART
#define SW_RX 9
#define SW_TX 8
SoftwareSerial mySerial(SW_RX, SW_TX);

// Motor pins
#define STEP1 2
#define DIR1 3
#define EN1 4

#define STEP2 5
#define DIR2 6
#define EN2 7

// Stepper objects
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);

// TMC drivers
TMC2209Stepper driver1(&mySerial, R_SENSE, 0);
TMC2209Stepper driver2(&mySerial, R_SENSE, 1);

void setup() {

  Serial.begin(115200);
  mySerial.begin(115200);

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);

  // Driver setup
  driver1.begin();
  driver1.rms_current(1400);
  driver1.microsteps(4);
  driver1.en_spreadCycle(true);

  driver2.begin();
  driver2.rms_current(1400);
  driver2.microsteps(4);
  driver2.en_spreadCycle(true);

  // Motion settings
  motor1.setMaxSpeed(90000);
  motor1.setAcceleration(30000);

  motor2.setMaxSpeed(90000);
  motor2.setAcceleration(30000);

  Serial.println("READY");
}

void loop() {

  motor1.run();
  motor2.run();

  readCommands();
}

void readCommands() {

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');

  if (cmd.startsWith("M1")) {
    long pos = cmd.substring(3).toInt();
    motor1.moveTo(pos);
  }

  else if (cmd.startsWith("M2")) {
    long pos = cmd.substring(3).toInt();
    motor2.moveTo(pos);
  }

  else if (cmd.startsWith("STOP")) {
    motor1.stop();
    motor2.stop();
  }

  else if (cmd.startsWith("EN")) {
    int e = cmd.substring(3).toInt();
    digitalWrite(EN1, !e);
    digitalWrite(EN2, !e);
  }

  else if (cmd.startsWith("POS")) {
    Serial.print("P1 ");
    Serial.println(motor1.currentPosition());

    Serial.print("P2 ");
    Serial.println(motor2.currentPosition());
  }
}