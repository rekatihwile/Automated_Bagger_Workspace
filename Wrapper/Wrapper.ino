#include <Servo.h>

Servo gripper;

// -------------------- SERVO --------------------
const int SERVO_PIN = 9;
const int MIN_SERVO_ANGLE = 5;
const int MAX_SERVO_ANGLE = 75;

// -------------------- STEPPER DRIVER --------------------
const int STEP_PIN = 3;
const int DIR_PIN  = 4;
const int EN_PIN   = 5;

// Adjust these if needed
const int STEP_PULSE_US = 800;  // slower = safer; try 500-1500 us
long stepperPosition = 0;       // software-tracked position in steps

String input = "";

void setup() {
  Serial.begin(9600);

  gripper.attach(SERVO_PIN);
  gripper.write(MIN_SERVO_ANGLE);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(EN_PIN, HIGH);   // disabled at startup

  Serial.println("READY");
  Serial.println("Commands:");
  Serial.println("A<angle>   servo angle 5-75, example A45");
  Serial.println("S<steps>   move stepper relative steps, example S200 or S-200");
  Serial.println("G<pos>     move stepper to absolute step position, example G1000");
  Serial.println("Z          zero stepper software position");
  Serial.println("E1         enable stepper");
  Serial.println("E0         disable stepper");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      handleCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char commandType = cmd[0];
  String valueString = cmd.substring(1);
  valueString.trim();

  if (commandType == 'A' || commandType == 'a') {
    handleServo(valueString);
  }
  else if (commandType == 'S' || commandType == 's') {
    handleStepperRelative(valueString);
  }
  else if (commandType == 'G' || commandType == 'g') {
    handleStepperAbsolute(valueString);
  }
  else if (commandType == 'Z' || commandType == 'z') {
    stepperPosition = 0;
    Serial.println("OK zeroed");
  }
  else if (commandType == 'E' || commandType == 'e') {
    handleStepperEnable(valueString);
  }
  else {
    Serial.println("ERR unknown_command");
  }
}

// -------------------- SERVO FUNCTIONS --------------------

void handleServo(String valueString) {
  if (!isInteger(valueString)) {
    Serial.println("ERR servo_nonnumeric");
    return;
  }

  int angle = valueString.toInt();

  if (angle < MIN_SERVO_ANGLE || angle > MAX_SERVO_ANGLE) {
    Serial.println("ERR servo_out_of_range");
    return;
  }

  gripper.write(angle);

  Serial.print("OK servo ");
  Serial.println(angle);
}

// -------------------- STEPPER FUNCTIONS --------------------

void handleStepperRelative(String valueString) {
  if (!isInteger(valueString)) {
    Serial.println("ERR stepper_nonnumeric");
    return;
  }

  long steps = valueString.toInt();
  moveStepperRelative(steps);

  Serial.print("OK stepper_pos ");
  Serial.println(stepperPosition);
}

void handleStepperAbsolute(String valueString) {
  if (!isInteger(valueString)) {
    Serial.println("ERR stepper_nonnumeric");
    return;
  }

  long targetPosition = valueString.toInt();
  long delta = targetPosition - stepperPosition;

  moveStepperRelative(delta);

  Serial.print("OK stepper_pos ");
  Serial.println(stepperPosition);
}

void handleStepperEnable(String valueString) {
  if (valueString == "1") {
    digitalWrite(EN_PIN, LOW);   // enabled
    Serial.println("OK stepper_enabled");
  }
  else if (valueString == "0") {
    digitalWrite(EN_PIN, HIGH);  // disabled
    Serial.println("OK stepper_disabled");
  }
  else {
    Serial.println("ERR enable_use_E1_or_E0");
  }
}

void moveStepperRelative(long steps) {
  if (steps == 0) return;

  digitalWrite(EN_PIN, LOW); // enable driver

  if (steps > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
    steps = -steps;
  }

  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }

  if (digitalRead(DIR_PIN) == HIGH) {
    stepperPosition += steps;
  } else {
    stepperPosition -= steps;
  }
}

// -------------------- VALIDATION --------------------

bool isInteger(String s) {
  if (s.length() == 0) return false;

  int startIndex = 0;

  if (s[0] == '-') {
    if (s.length() == 1) return false;
    startIndex = 1;
  }

  for (int i = startIndex; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }

  return true;
}