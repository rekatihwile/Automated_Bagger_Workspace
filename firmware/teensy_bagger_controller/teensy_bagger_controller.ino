#include <AccelStepper.h>
#include <TMCStepper.h>
#include <Servo.h>

// ============================================================================
// Automated Grocery Bagger - Teensy Low-Level Controller
// ============================================================================
// Preferred high-level command from Python dev app:
//   Q <J1_steps> <J2_steps> <J3_steps> <J4_steps>
// Example:
//   Q 1200 -4500 3000 0
//
// This firmware intentionally does NOT solve IK. Python owns:
//   workspace xyz -> IK physical joints -> transmission coupling -> motor steps
// Teensy owns:
//   stepper motion, enable/disable, servo, status, safety stop, position feedback
// ============================================================================

#define FIRMWARE_NAME "BAGGER_TEENSY_CONTROLLER"
#define FIRMWARE_VERSION "1.1.0"

#define R_SENSE 0.11f

// --------------------
// USER SETTINGS
// --------------------

// Serial links
#define PC_BAUD 115200
#define TMC_BAUD 115200
#define UART_PORT Serial1

// STEP/DIR/ENABLE pins
#define J1_STEP 2
#define J1_DIR 3
#define J1_EN 4

#define J2_STEP 5
#define J2_DIR 6
#define J2_EN 7

#define J3_STEP 8
#define J3_DIR 9
#define J3_EN 10

#define J4_STEP 11
#define J4_DIR 12
#define J4_EN 13

#define SERVO_PIN 14

// Servo limits for claw
const int SERVO_MIN_DEG = 10;
const int SERVO_MAX_DEG = 70;
const int SERVO_DEFAULT_DEG = 40;
const int SERVO_OPEN_DEG = 70;
const int SERVO_CLOSED_DEG = 25;

// TMC current limits [mA RMS]
const uint16_t J1_CURRENT_MA = 1400;
const uint16_t J2_CURRENT_MA = 1400;
const uint16_t J3_CURRENT_MA = 1400;
const uint16_t J4_CURRENT_MA = 800;

// Microsteps must match Python BaggerTransmission.microsteps
const uint16_t MICROSTEPS = 8;
const float FULL_STEPS_PER_REV = 200.0f;
const float STEPS_PER_MOTOR_DEG = (FULL_STEPS_PER_REV * (float)MICROSTEPS) / 360.0f;

// Motion tuning [steps/s, steps/s^2]
const float J1_MAX_SPEED = 25000.0f;
const float J1_ACCEL = 12000.0f;

const float J2_MAX_SPEED = 25000.0f;
const float J2_ACCEL = 12000.0f;

const float J3_MAX_SPEED = 12000.0f;
const float J3_ACCEL = 6000.0f;

const float J4_MAX_SPEED = 40000.0f;
const float J4_ACCEL = 20000.0f;

// Software limits in controller steps.
// These are intentionally wide placeholders. Tighten them after calibration.
long MIN_POS[4] = {-500000L, -500000L, -500000L, -500000L};
long MAX_POS[4] = { 500000L,  500000L,  500000L,  500000L};

// --------------------
// STEPPER OBJECTS
// --------------------

AccelStepper J1(AccelStepper::DRIVER, J1_STEP, J1_DIR);
AccelStepper J2(AccelStepper::DRIVER, J2_STEP, J2_DIR);
AccelStepper J3(AccelStepper::DRIVER, J3_STEP, J3_DIR);
AccelStepper J4(AccelStepper::DRIVER, J4_STEP, J4_DIR);
AccelStepper* JOINTS[4] = {&J1, &J2, &J3, &J4};

const int EN_PINS[4] = {J1_EN, J2_EN, J3_EN, J4_EN};

// --------------------
// DRIVER OBJECTS
// --------------------
// This assumes the TMC2209 address pins are configured as 0,1,2,3.
// If UART config fails, verify the MS1/MS2 address wiring on each driver.

TMC2209Stepper driver1(&UART_PORT, R_SENSE, 0);
TMC2209Stepper driver2(&UART_PORT, R_SENSE, 1);
TMC2209Stepper driver3(&UART_PORT, R_SENSE, 2);
TMC2209Stepper driver4(&UART_PORT, R_SENSE, 3);

// --------------------
// SERVO
// --------------------

Servo clawServo;
int servoAngle = SERVO_DEFAULT_DEG;

// --------------------
// STATE
// --------------------

bool enabled = true;
bool motionWasActive = false;
bool reportDoneOnArrival = true;

char rxBuffer[160];
uint8_t rxIndex = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(PC_BAUD);
  UART_PORT.begin(TMC_BAUD);

  pinMode(J1_EN, OUTPUT);
  pinMode(J2_EN, OUTPUT);
  pinMode(J3_EN, OUTPUT);
  pinMode(J4_EN, OUTPUT);

  enableMotors(true);

  setupDriver(driver1, J1_CURRENT_MA);
  setupDriver(driver2, J2_CURRENT_MA);
  setupDriver(driver3, J3_CURRENT_MA);
  setupDriver(driver4, J4_CURRENT_MA);

  setupStepper(J1, J1_MAX_SPEED, J1_ACCEL);
  setupStepper(J2, J2_MAX_SPEED, J2_ACCEL);
  setupStepper(J3, J3_MAX_SPEED, J3_ACCEL);
  setupStepper(J4, J4_MAX_SPEED, J4_ACCEL);

  clawServo.attach(SERVO_PIN);
  setServoAngle(SERVO_DEFAULT_DEG);

  zeroAllJoints();

  Serial.print("READY ");
  Serial.print(FIRMWARE_NAME);
  Serial.print(" ");
  Serial.println(FIRMWARE_VERSION);
  printStatus();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  runSteppers();
  readSerialNonBlocking();
  maybeReportDone();
}

// ============================================================================
// SETUP HELPERS
// ============================================================================

void setupDriver(TMC2209Stepper &driver, uint16_t current_mA) {
  driver.begin();
  driver.rms_current(current_mA);
  driver.microsteps(MICROSTEPS);
  driver.en_spreadCycle(true);
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
}

void setupStepper(AccelStepper &motor, float maxSpeed, float accel) {
  motor.setMaxSpeed(maxSpeed);
  motor.setAcceleration(accel);
}

void runSteppers() {
  J1.run();
  J2.run();
  J3.run();
  J4.run();
}

// ============================================================================
// ENABLE / ZERO / STOP
// ============================================================================

void enableMotors(bool state) {
  enabled = state;
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], !state);  // TMC enable is usually active-low
  }
  Serial.print("OK EN ");
  Serial.println(state ? 1 : 0);
}

void zeroJoint(uint8_t index) {
  if (index >= 4) {
    Serial.println("ERR BAD_JOINT");
    return;
  }
  JOINTS[index]->setCurrentPosition(0);
  JOINTS[index]->moveTo(0);
  motionWasActive = false;
  Serial.print("OK ZERO J");
  Serial.println(index + 1);
}

void zeroAllJoints() {
  for (uint8_t i = 0; i < 4; i++) {
    JOINTS[i]->setCurrentPosition(0);
    JOINTS[i]->moveTo(0);
  }
  motionWasActive = false;
  Serial.println("OK ZERO ALL");
}

void setCurrentPositionOne(uint8_t index, long pos) {
  if (index >= 4) {
    Serial.println("ERR BAD_JOINT");
    return;
  }
  JOINTS[index]->setCurrentPosition(pos);
  JOINTS[index]->moveTo(pos);
  motionWasActive = false;
  Serial.print("OK SETPOS J");
  Serial.print(index + 1);
  Serial.print(' ');
  Serial.println(pos);
}

void setCurrentPositions(long pos[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    JOINTS[i]->setCurrentPosition(pos[i]);
    JOINTS[i]->moveTo(pos[i]);
  }
  motionWasActive = false;
  Serial.print("OK SETPOS ");
  printPositionsCompact();
}

void hardStopAll() {
  // stop() decelerates according to AccelStepper acceleration.
  // For true power-cut ESTOP, use external hardware power cutoff.
  for (uint8_t i = 0; i < 4; i++) {
    JOINTS[i]->stop();
  }
  Serial.println("OK STOP");
}

void immediateFreezeAll() {
  // Software emergency-style freeze: current target becomes current position.
  // This is not a replacement for a physical E-stop.
  for (uint8_t i = 0; i < 4; i++) {
    long p = JOINTS[i]->currentPosition();
    JOINTS[i]->setCurrentPosition(p);
    JOINTS[i]->moveTo(p);
  }
  motionWasActive = false;
  Serial.println("OK FREEZE");
}

// ============================================================================
// MOTION COMMANDS
// ============================================================================

bool checkLimits(long targets[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    if (targets[i] < MIN_POS[i] || targets[i] > MAX_POS[i]) {
      Serial.print("ERR LIMIT J");
      Serial.print(i + 1);
      Serial.print(" TARGET ");
      Serial.println(targets[i]);
      return false;
    }
  }
  return true;
}

void moveAll(long targets[4]) {
  if (!enabled) {
    Serial.println("ERR DISABLED");
    return;
  }

  if (!checkLimits(targets)) {
    return;
  }

  J1.moveTo(targets[0]);
  J2.moveTo(targets[1]);
  J3.moveTo(targets[2]);
  J4.moveTo(targets[3]);

  motionWasActive = true;

  Serial.print("OK MOVE ");
  Serial.print(targets[0]); Serial.print(' ');
  Serial.print(targets[1]); Serial.print(' ');
  Serial.print(targets[2]); Serial.print(' ');
  Serial.println(targets[3]);
}

void moveSingle(uint8_t index, long target) {
  if (index >= 4) {
    Serial.println("ERR BAD_JOINT");
    return;
  }
  long targets[4] = {
    J1.targetPosition(),
    J2.targetPosition(),
    J3.targetPosition(),
    J4.targetPosition()
  };
  targets[index] = target;
  moveAll(targets);
}

bool isBusy() {
  for (uint8_t i = 0; i < 4; i++) {
    if (JOINTS[i]->distanceToGo() != 0) return true;
  }
  return false;
}

void maybeReportDone() {
  bool busy = isBusy();
  if (motionWasActive && !busy && reportDoneOnArrival) {
    motionWasActive = false;
    Serial.print("DONE ");
    printPositionsCompact();
  }
}

// ============================================================================
// SERVO
// ============================================================================

void setServoAngle(int angle) {
  angle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  servoAngle = angle;
  clawServo.write(servoAngle);
  Serial.print("OK SERVO ");
  Serial.println(servoAngle);
}

// ============================================================================
// SERIAL PARSER
// ============================================================================

void readSerialNonBlocking() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';
      if (rxIndex > 0) {
        handleCommand(rxBuffer);
      }
      rxIndex = 0;
      return;
    }

    if (rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = c;
    } else {
      rxIndex = 0;
      Serial.println("ERR LINE_TOO_LONG");
    }
  }
}

void handleCommand(char* line) {
  trimLeadingSpaces(line);
  if (line[0] == '\0') return;

  char* cmd = strtok(line, " ,\t");
  if (cmd == NULL) return;
  uppercase(cmd);

  if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
  }
  else if (strcmp(cmd, "HELP") == 0) {
    printHelp();
  }
  else if (strcmp(cmd, "Q") == 0 || strcmp(cmd, "MOVE") == 0) {
    long targets[4];
    if (!parseFourLongs(targets)) {
      Serial.println("ERR BAD_MOVE_ARGS");
      return;
    }
    moveAll(targets);
  }
  else if (cmd[0] == 'J' && cmd[1] >= '1' && cmd[1] <= '4' && cmd[2] == '\0') {
    char* arg = strtok(NULL, " ,\t");
    if (arg == NULL) {
      Serial.println("ERR BAD_J_ARGS");
      return;
    }
    moveSingle((uint8_t)(cmd[1] - '1'), atol(arg));
  }
  else if (strcmp(cmd, "SERVO") == 0) {
    char* arg = strtok(NULL, " ,\t");
    if (arg == NULL) {
      Serial.println("ERR BAD_SERVO_ARGS");
      return;
    }
    setServoAngle(atoi(arg));
  }
  else if (strcmp(cmd, "CLAW") == 0) {
    char* arg = strtok(NULL, " ,\t");
    if (arg == NULL) {
      Serial.println("ERR BAD_CLAW_ARGS");
      return;
    }
    uppercase(arg);
    if (strcmp(arg, "OPEN") == 0) setServoAngle(SERVO_OPEN_DEG);
    else if (strcmp(arg, "CLOSE") == 0 || strcmp(arg, "CLOSED") == 0) setServoAngle(SERVO_CLOSED_DEG);
    else Serial.println("ERR BAD_CLAW_STATE");
  }
  else if (strcmp(cmd, "EN") == 0) {
    char* arg = strtok(NULL, " ,\t");
    if (arg == NULL) {
      Serial.println("ERR BAD_EN_ARGS");
      return;
    }
    enableMotors(atoi(arg) != 0);
  }
  else if (strcmp(cmd, "STOP") == 0) {
    hardStopAll();
  }
  else if (strcmp(cmd, "FREEZE") == 0 || strcmp(cmd, "ESTOP") == 0) {
    immediateFreezeAll();
  }
  else if (strcmp(cmd, "ZERO") == 0) {
    handleZero();
  }
  else if (strcmp(cmd, "SETPOS") == 0 || strcmp(cmd, "SET_POSITION") == 0) {
    handleSetPos();
  }
  else if (strcmp(cmd, "SETANGLE") == 0 || strcmp(cmd, "SET_ANGLE") == 0) {
    handleSetAngle();
  }
  else if (strcmp(cmd, "POS") == 0) {
    printPositions();
  }
  else if (strcmp(cmd, "STAT") == 0 || strcmp(cmd, "STATUS") == 0) {
    printStatus();
  }
  else if (strcmp(cmd, "DONE") == 0) {
    reportDoneOnArrival = true;
    Serial.println("OK DONE_REPORT 1");
  }
  else if (strcmp(cmd, "NODONE") == 0) {
    reportDoneOnArrival = false;
    Serial.println("OK DONE_REPORT 0");
  }
  else if (strcmp(cmd, "SETMAX") == 0) {
    handleSetMax();
  }
  else if (strcmp(cmd, "SETACC") == 0) {
    handleSetAcc();
  }
  else if (strcmp(cmd, "LIMIT") == 0) {
    handleLimit();
  }
  else {
    Serial.print("ERR UNKNOWN_CMD ");
    Serial.println(cmd);
  }
}

bool parseFourLongs(long out[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    char* arg = strtok(NULL, " ,\t");
    if (arg == NULL) return false;
    out[i] = atol(arg);
  }
  return true;
}


bool parseJointIndexToken(char* token, uint8_t &indexOut) {
  if (token == NULL) return false;
  uppercase(token);

  int n = -1;
  if (token[0] == 'J' && token[1] >= '1' && token[1] <= '4' && token[2] == '\0') {
    n = token[1] - '0';
  } else {
    n = atoi(token);
  }

  if (n < 1 || n > 4) return false;
  indexOut = (uint8_t)(n - 1);
  return true;
}

void handleZero() {
  char* arg = strtok(NULL, " ,\t");
  if (arg == NULL) {
    zeroAllJoints();
    return;
  }

  uppercase(arg);
  if (strcmp(arg, "ALL") == 0) {
    zeroAllJoints();
    return;
  }

  uint8_t idx = 0;
  if (!parseJointIndexToken(arg, idx)) {
    Serial.println("ERR ZERO_ARGS");
    return;
  }
  zeroJoint(idx);
}

void handleSetPos() {
  char* first = strtok(NULL, " ,\t");
  if (first == NULL) {
    Serial.println("ERR SETPOS_ARGS");
    return;
  }

  uint8_t idx = 0;
  char firstCopy[12];
  strncpy(firstCopy, first, sizeof(firstCopy) - 1);
  firstCopy[sizeof(firstCopy) - 1] = '\0';

  // Joint-specific SETPOS must use J1/J2/J3/J4. This avoids confusing
  // a valid all-joint target like "SETPOS 1 2 3 4" with "SETPOS J1 2".
  if (firstCopy[0] == 'J' || firstCopy[0] == 'j') {
    if (!parseJointIndexToken(firstCopy, idx)) {
      Serial.println("ERR BAD_JOINT");
      return;
    }
    char* val = strtok(NULL, " ,\t");
    if (val == NULL) {
      Serial.println("ERR SETPOS_J_ARGS");
      return;
    }
    setCurrentPositionOne(idx, atol(val));
    return;
  }

  long pos[4];
  pos[0] = atol(first);
  for (uint8_t i = 1; i < 4; i++) {
    char* arg = strtok(NULL, " ,\t");
    if (arg == NULL) {
      Serial.println("ERR SETPOS_4_ARGS");
      return;
    }
    pos[i] = atol(arg);
  }
  setCurrentPositions(pos);
}

void handleSetAngle() {
  char* jointToken = strtok(NULL, " ,\t");
  char* angleToken = strtok(NULL, " ,\t");
  if (jointToken == NULL || angleToken == NULL) {
    Serial.println("ERR SETANGLE_ARGS");
    return;
  }

  uint8_t idx = 0;
  if (!parseJointIndexToken(jointToken, idx)) {
    Serial.println("ERR BAD_JOINT");
    return;
  }

  float motorDeg = atof(angleToken);
  long targetSteps = lround(motorDeg * STEPS_PER_MOTOR_DEG);
  moveSingle(idx, targetSteps);
}

void handleSetMax() {
  char* j = strtok(NULL, " ,\t");
  char* v = strtok(NULL, " ,\t");
  if (j == NULL || v == NULL) {
    Serial.println("ERR SETMAX_ARGS");
    return;
  }
  int idx = atoi(j) - 1;
  if (idx < 0 || idx > 3) {
    Serial.println("ERR BAD_JOINT");
    return;
  }
  JOINTS[idx]->setMaxSpeed(atof(v));
  Serial.println("OK SETMAX");
}

void handleSetAcc() {
  char* j = strtok(NULL, " ,\t");
  char* v = strtok(NULL, " ,\t");
  if (j == NULL || v == NULL) {
    Serial.println("ERR SETACC_ARGS");
    return;
  }
  int idx = atoi(j) - 1;
  if (idx < 0 || idx > 3) {
    Serial.println("ERR BAD_JOINT");
    return;
  }
  JOINTS[idx]->setAcceleration(atof(v));
  Serial.println("OK SETACC");
}

void handleLimit() {
  char* j = strtok(NULL, " ,\t");
  char* lo = strtok(NULL, " ,\t");
  char* hi = strtok(NULL, " ,\t");
  if (j == NULL || lo == NULL || hi == NULL) {
    Serial.println("ERR LIMIT_ARGS");
    return;
  }
  int idx = atoi(j) - 1;
  if (idx < 0 || idx > 3) {
    Serial.println("ERR BAD_JOINT");
    return;
  }
  MIN_POS[idx] = atol(lo);
  MAX_POS[idx] = atol(hi);
  Serial.println("OK LIMIT");
}

// ============================================================================
// PRINT HELPERS
// ============================================================================

void printPositionsCompact() {
  Serial.print(J1.currentPosition()); Serial.print(' ');
  Serial.print(J2.currentPosition()); Serial.print(' ');
  Serial.print(J3.currentPosition()); Serial.print(' ');
  Serial.println(J4.currentPosition());
}

void printPositions() {
  Serial.print("POS ");
  printPositionsCompact();
  Serial.print("TGT ");
  Serial.print(J1.targetPosition()); Serial.print(' ');
  Serial.print(J2.targetPosition()); Serial.print(' ');
  Serial.print(J3.targetPosition()); Serial.print(' ');
  Serial.println(J4.targetPosition());
  Serial.print("SERVO ");
  Serial.println(servoAngle);
}

void printStatus() {
  Serial.print("STAT ENABLED ");
  Serial.print(enabled ? 1 : 0);
  Serial.print(" BUSY ");
  Serial.print(isBusy() ? 1 : 0);
  Serial.print(" SERVO ");
  Serial.print(servoAngle);
  Serial.print(" MICROSTEPS ");
  Serial.println(MICROSTEPS);
}

void printHelp() {
  Serial.println("CMDS:");
  Serial.println("  Q s1 s2 s3 s4       absolute coordinated move in steps");
  Serial.println("  MOVE s1 s2 s3 s4    same as Q");
  Serial.println("  J1 s                single joint absolute move");
  Serial.println("  SERVO deg           servo angle");
  Serial.println("  CLAW OPEN|CLOSE     preset servo positions");
  Serial.println("  EN 1|0              enable/disable motors");
  Serial.println("  STOP                decelerating stop");
  Serial.println("  FREEZE              immediate software freeze");
  Serial.println("  ZERO                set all current step positions to zero");
  Serial.println("  ZERO Jn             set one current step position to zero");
  Serial.println("  SETPOS s1 s2 s3 s4  set current step counters, no motion");
  Serial.println("  SETPOS Jn s         set one current step counter, no motion");
  Serial.println("  SETANGLE Jn deg     move one motor to motor-space angle in degrees");
  Serial.println("  POS                 print positions and targets");
  Serial.println("  STAT                print enabled/busy status");
  Serial.println("  SETMAX j value      set max speed for joint j");
  Serial.println("  SETACC j value      set acceleration for joint j");
  Serial.println("  LIMIT j min max     set software step limits for joint j");
}

// ============================================================================
// STRING HELPERS
// ============================================================================

void uppercase(char* s) {
  while (*s) {
    if (*s >= 'a' && *s <= 'z') *s = *s - 32;
    s++;
  }
}

void trimLeadingSpaces(char* s) {
  uint8_t i = 0;
  while (s[i] == ' ' || s[i] == '\t') i++;
  if (i == 0) return;

  uint8_t j = 0;
  while (s[i] != '\0') {
    s[j++] = s[i++];
  }
  s[j] = '\0';
}
