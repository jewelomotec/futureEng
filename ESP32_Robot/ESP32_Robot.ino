#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

// ---- WiFi tuning console ----
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>


// ==========================================
//           PIN & HARDWARE DEFINITIONS
// ==========================================
#define SERVO_PIN 13


// Left Motor Pins
#define MOTOR_IN1  25
#define MOTOR_IN2  26
#define MOTOR_PWM  33


// I2C Multiplexer Channels (HW-617)
#define MUX_CH_LEFT   0
#define MUX_CH_CENTER 1
#define MUX_CH_RIGHT  2
#define MUX_CH_BNO    4


#define TFLUNA_I2C_ADDR 0x10


// ==========================================
//          CONFIGURABLE VARIABLES
// ==========================================
int STRAIGHT_SPEED = 80;  // Cruise speed for driving straight (0-255)
int TURN_SPEED     = 80;  // Kept for WiFi tuner compatibility
int BACKWARD_SPEED = -80; // Speed used during the reversing arc

const int RAMP_START_SPEED = 30;
const int RAMP_STEP = 30;
const unsigned long RAMP_DURATION_MS = 2000;


int SERVO_CENTER   = 117;   // Dead-center steering alignment
int DIFF = 25;
int SERVO_MAX_LEFT = SERVO_CENTER + DIFF;  // Physical mechanical limit for left turn
int SERVO_MAX_RIGHT= SERVO_CENTER - DIFF;  // Physical mechanical limit for right turn

// Change to true if your steering corrections move backwards during testing
const bool INVERT_STEERING = true;

float STEERING_KP  = 1.2;
float STEERING_KI  = 0.02;
float STEERING_KD  = 0.15;
const float HEADING_DEADBAND = 1.5;   // degrees — ignore jitter smaller than this
const int   MAX_STEER_CORRECTION = 20; // clamp — no single correction can swing the servo too hard
const int MAX_TURNS      = 12;   // Total number of turns allowed before tracking the final stop distance

unsigned long turnCooldownUntil = 0;   // timestamp until which obstacle checks are ignored
// ---- Obstacle avoidance (serial commands from the Pi) ----
bool avoidDirectionRight = true;               // true = swerve right, false = swerve left
unsigned long lastObstacleCmd = 0;             // timestamp of last RED/GREEN command
const unsigned long OBSTACLE_TIMEOUT_MS = 5000; // auto-clear after 5s of no command
const unsigned long TURN_COOLDOWN_MS = 1000; // how long to ignore front-wall checks after a turn

// Cardinal heading snap table — every commanded turn target is forced onto one of these,
// so sensor noise/drift can never leave the robot aiming at something like 250°.
const float CARDINAL_HEADINGS[4] = {0.0, 90.0, 180.0, 270.0};

// ==========================================
//      FRONT-DISTANCE TURN TRIGGER
// ==========================================
// Start a reversing-arc turn once the front (center) LiDAR stays this close.
int FRONT_TURN_DISTANCE = 15;              // cm — start the wall reverse-arc once front gets this close
bool frontConditionActive = false;         // true while front has been continuously "close"
unsigned long frontConditionStartTime = 0; // millis() timestamp when it first became true
const unsigned long FRONT_CONFIRM_MS = 150; // debounce so one noisy reading doesn't trigger a turn

// ==========================================
//           REVERSING ARC TURN
// ==========================================
// Instead of a three-point turn, a wall trigger backs the robot up with the
// servo cranked by ARC_SERVO_ANGLE. The IMU is watched until heading is
// within ARC_EXIT_THRESHOLD of the target cardinal, then wheels center and
// straight driving resumes.
//
// Tune these for lane width and how square the exit needs to be:
int ARC_SERVO_ANGLE = 20;              // degrees off center — how sharp the reverse arc is
float ARC_EXIT_THRESHOLD = 8.0;        // degrees — how precisely it must face the target before exiting
unsigned long ARC_PAUSE_MS = 500;      // stand still after the 15 cm / 150 ms confirm, before reversing
unsigned long ARC_MIN_MS = 400;        // prevent an instant exit if heading is already close
unsigned long ARC_MAX_MS = 4000;       // safety timeout — force-exit the arc even if still off heading

enum TurnPhase { PHASE_PAUSE, PHASE_REVERSE };
TurnPhase currentTurnPhase = PHASE_PAUSE;
unsigned long turnPhaseStartTime = 0;
unsigned long arcStartTime = 0;

// ==========================================
//     PI WAYPOINT (STOP / WAYPOINT / REVERSE)
// ==========================================
// Pi sends STOP then WAYPOINT,color,xa,ya,xc,yc,R,theta,arclen
float WHEELBASE_CM = 18.0;                 // used to turn radius into a servo angle
unsigned long WAYPOINT_PAUSE_MS = 0;       // 0 = no stand-still; drive to C as soon as WAYPOINT arrives
float WAYPOINT_EXIT_DEG = 8.0;             // IMU heading error that counts as "arrived"
unsigned long WAYPOINT_MIN_MS = 250;
unsigned long WAYPOINT_MAX_MS = 4000;
float ESTIMATED_FWD_CMS = 30.0;            // rough cm/s at STRAIGHT_SPEED — backup timer

bool waypointReady = false;
float waypointStartHeading = 0.0;
float waypointTargetHeading = 0.0;
float waypointRadiusCm = 0.0;
float waypointThetaDeg = 0.0;
float waypointArcLenCm = 0.0;
int waypointServoAngle = 90;
unsigned long piHoldStart = 0;
unsigned long waypointArcStart = 0;
unsigned long lastPiCmd = 0;


// ==========================================
//        WIFI TUNING CONSOLE SETTINGS
// ==========================================
const char* AP_SSID     = "RobotTuner";
const char* AP_PASSWORD = "tunemybot";   // must be 8+ characters for WPA2
// Once connected to this WiFi network, open http://robot.local
// or http://192.168.4.1 if robot.local doesn't resolve on your phone.

WebServer server(80);
Preferences prefs;


// ==========================================
//          GLOBAL STATE VARIABLES
// ==========================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringServo;


enum RobotState {
  DRIVING_STRAIGHT,
  TURNING,
  ROBOT_STOPPED,
  OBSTACLE_AVOIDING,
  PI_HOLD,
  WAYPOINT_ARC,
  PI_REVERSE
};
RobotState currentState = DRIVING_STRAIGHT;


float straightTargetHeading = 0.0;
float turnTargetHeading     = 0.0;
bool isTurningLeft          = false;


int totalTurnsCount         = 0;
bool hasTurnedOnce          = false; // Becomes true permanently on the first turn
bool lockedDirectionLeft    = false; // Remembers if our layout is strictly Left or Right


// Global telemetry variables
int16_t currentLeftDist     = -1;
int16_t currentCenterDist   = -1;
int16_t currentRightDist    = -1;
int finalServoAngle         = 90;
float headingError          = 0.0;
float angleDifference       = 0.0;
float lastHeadingError       = 0.0;
float integralError          = 0.0;   // running total of error over time
unsigned long lastHeadingTime = 0;    // for time-based derivative

// ---- Speed ramp-up state ----
unsigned long driveStartTime = 0;   // when the very first straight-drive began
bool rampActive = false;            // true while we're still ramping up from a stand-still
bool rampArmedForThisPhase = false; // true once the ramp has been (re)started for the current phase

// ==========================================
//            I2C & SENSOR FUNCTIONS
// ==========================================
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}


int16_t getLunaDistance(uint8_t channel) {
  selectMuxChannel(channel);
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return -1;
  Wire.requestFrom(TFLUNA_I2C_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    return (lowByte + (highByte << 8));
  }
  return -1;
}


float getCurrentHeading() {
  selectMuxChannel(MUX_CH_BNO);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float filteredHeading = 0.0;
bool headingInitialized = false;

// --- Stop condition (L & R < 100cm, front < 150cm, held for 1 second) ---
bool stopConditionActive             = false;
unsigned long stopConditionStartTime = 0;
const unsigned long STOP_CONFIRM_MS  = 1000;

float getSmoothedHeading() {
  float raw = getCurrentHeading();
  if (!headingInitialized) {
    filteredHeading = raw;
    headingInitialized = true;
    return filteredHeading;
  }

  // Find the shortest angular distance from filtered to raw (handles the 0/360 wrap)
  float diff = raw - filteredHeading;
  if (diff > 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;

  filteredHeading += 0.2 * diff;   // same 0.8/0.2 blend, but wrap-safe
  if (filteredHeading < 0.0)    filteredHeading += 360.0;
  if (filteredHeading >= 360.0) filteredHeading -= 360.0;

  return filteredHeading;
}

// ==========================================
//        CARDINAL HEADING SNAP HELPER
// ==========================================
// Rounds any angle to the nearest of 0/90/180/270, wrapping correctly around 0/360.
float snapToCardinal(float angle) {
  angle = fmod(angle, 360.0);
  if (angle < 0.0) angle += 360.0;

  float best = CARDINAL_HEADINGS[0];
  float bestDiff = 999.0;

  for (int i = 0; i < 4; i++) {
    float diff = fabs(angle - CARDINAL_HEADINGS[i]);
    if (diff > 180.0) diff = 360.0 - diff; // shortest wrap-around distance
    if (diff < bestDiff) {
      bestDiff = diff;
      best = CARDINAL_HEADINGS[i];
    }
  }
  return best;
}

// Computes a turn target: takes the raw current heading +/-90 depending on direction,
// then snaps it onto the nearest cardinal so the robot always ends up aimed at
// exactly 0, 90, 180, or 270 — never something drifted like 250.
float computeTurnTarget(float currentHeading, bool turningLeft) {
  float raw = turningLeft ? (currentHeading - 90.0) : (currentHeading + 90.0);
  raw = fmod(raw, 360.0);
  if (raw < 0.0) raw += 360.0;
  return snapToCardinal(raw);
}

float shortestAngleDiff(float fromHeading, float toHeading) {
  float diff = fromHeading - toHeading;
  if (diff > 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}

// ==========================================
//            ACTUATOR FUNCTIONS
// ==========================================
void setMotorOutput(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    speed = -speed; // Convert negative to positive for PWM calculation
  }
  analogWrite(MOTOR_PWM, constrain(speed, 0, 255));
}

// ==========================================
//          SPEED RAMP-UP HELPER
// ==========================================
// On the very first start (t=0), speed climbs in RAMP_STEP increments from
// RAMP_START_SPEED up to STRAIGHT_SPEED over RAMP_DURATION_MS, so the car
// doesn't dump full torque instantly and pop a wheelie off the line.
int getRampedSpeed(int targetSpeed) {
  if (!rampActive) return targetSpeed;

  unsigned long elapsed = millis() - driveStartTime;

  if (elapsed >= RAMP_DURATION_MS) {
    rampActive = false;
    return targetSpeed;
  }

  int numSteps = max(1, (targetSpeed - RAMP_START_SPEED) / RAMP_STEP);
  unsigned long stepDuration = RAMP_DURATION_MS / numSteps;

  int stepIndex = elapsed / stepDuration;
  int speed = RAMP_START_SPEED + stepIndex * RAMP_STEP;

  return constrain(speed, RAMP_START_SPEED, targetSpeed);
}


// ==========================================
//             BEHAVIOR LOGIC MODES
// ==========================================

// Starts a reversing-arc turn once the front (center) LiDAR reports we're
// within FRONT_TURN_DISTANCE cm, held continuously for FRONT_CONFIRM_MS.
void checkFrontObstacle() {
  if (millis() < turnCooldownUntil) return;   // skip checking right after a turn
  if (currentCenterDist <= 0) { frontConditionActive = false; return; }

  if (currentCenterDist < FRONT_TURN_DISTANCE) {
    if (!frontConditionActive) {
      frontConditionActive = true;
      frontConditionStartTime = millis();
      return;
    }

    if (millis() - frontConditionStartTime < FRONT_CONFIRM_MS) return;

    // Confirmed — decide which way to turn.
    bool turnLeft;
    if (hasTurnedOnce) {
      // HARD LOCK: once we've committed to a direction, always turn that way.
      turnLeft = lockedDirectionLeft;
    } else {
      // FIRST TURN: pick whichever side currently has more room.
      turnLeft = (currentLeftDist > currentRightDist);
      hasTurnedOnce = true;
      lockedDirectionLeft = turnLeft;

      if (turnLeft) {
        Serial.println("!!! LAYOUT INITIALIZED: PERMANENT LEFT TURN ONLY LOCK ACTIVATED !!!");
      } else {
        Serial.println("!!! LAYOUT INITIALIZED: PERMANENT RIGHT TURN ONLY LOCK ACTIVATED !!!");
      }
    }

    float currentHeading = getSmoothedHeading();
    isTurningLeft = turnLeft;
    turnTargetHeading = computeTurnTarget(currentHeading, turnLeft);

    currentTurnPhase = PHASE_PAUSE;
    turnPhaseStartTime = millis();
    arcStartTime = 0;
    currentState = TURNING;
    frontConditionActive = false;
    rampArmedForThisPhase = false;
  } else {
    frontConditionActive = false; // reset — condition dropped out before confirm delay completed
  }
}


void driveStraightMode(float currentHeading) {
  if (!rampArmedForThisPhase) { driveStartTime = millis(); rampActive = true; rampArmedForThisPhase = true; }
  setMotorOutput(getRampedSpeed(STRAIGHT_SPEED));

  float rawError = straightTargetHeading - currentHeading;
  if (rawError > 180.0)  rawError -= 360.0;
  if (rawError < -180.0) rawError += 360.0;

  headingError = rawError; // keep raw value for telemetry + rate calc (no snapping to 0)

  // D term: real rate of change (degrees per second), computed from the RAW error
  // so it never sees a fake jump caused by the deadband
  unsigned long now = millis();
  float dt = (now - lastHeadingTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;

  float errorRate = (rawError - lastHeadingError) / dt;

  float pTermInput = (abs(rawError) < HEADING_DEADBAND) ? 0.0 : rawError;

  // Only accumulate the integral when there's a real (non-deadbanded) error —
  // this stops it from slowly building up due to sensor noise while "on target"
  if (abs(rawError) >= HEADING_DEADBAND) {
    integralError += rawError * dt;
  }

  // Clamp the integral so it can't build up forever and cause a huge overcorrection
  const float MAX_INTEGRAL = 50.0;   // degrees·seconds
  integralError = constrain(integralError, -MAX_INTEGRAL, MAX_INTEGRAL);

  int steeringCorrection = (int)round(pTermInput * STEERING_KP + integralError * STEERING_KI + errorRate * STEERING_KD);
  steeringCorrection = constrain(steeringCorrection, -MAX_STEER_CORRECTION, MAX_STEER_CORRECTION);

  lastHeadingError = rawError;
  lastHeadingTime = now;

  if (INVERT_STEERING) {
    finalServoAngle = SERVO_CENTER + steeringCorrection;
  } else {
    finalServoAngle = SERVO_CENTER - steeringCorrection;
  }
  finalServoAngle = constrain(finalServoAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  steeringServo.write(finalServoAngle);
}


// Servo angle for the reversing arc.
// While reversing, steering geometry flips: crank OPPOSITE the intended turn
// so the rear swings toward the new heading. Magnitude is ARC_SERVO_ANGLE
// (not full mechanical lock) so you can tune for lane width.
int reversingArcServoAngle() {
  int offset = constrain(ARC_SERVO_ANGLE, 0, DIFF);
  int leftExtreme  = INVERT_STEERING ? (SERVO_CENTER - offset) : (SERVO_CENTER + offset);
  int rightExtreme = INVERT_STEERING ? (SERVO_CENTER + offset) : (SERVO_CENTER - offset);
  int reverseCranked = isTurningLeft ? rightExtreme : leftExtreme;
  return constrain(reverseCranked, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

void finishArcTurn() {
  totalTurnsCount++;
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  straightTargetHeading = turnTargetHeading;
  currentState = DRIVING_STRAIGHT;
  currentTurnPhase = PHASE_PAUSE;
  rampArmedForThisPhase = false;
  turnCooldownUntil = millis() + TURN_COOLDOWN_MS;
  integralError = 0.0;
  lastHeadingError = 0.0;
  lastHeadingTime = millis();
}

// After the 15 cm / 150 ms confirm: stand still (ARC_PAUSE_MS), then reverse
// with the servo cranked until heading is on the target cardinal.
void executeTurnMode(float currentHeading) {
  unsigned long now = millis();

  angleDifference = shortestAngleDiff(currentHeading, turnTargetHeading);

  if (currentTurnPhase == PHASE_PAUSE) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
    finalServoAngle = SERVO_CENTER;

    if (now - turnPhaseStartTime >= ARC_PAUSE_MS) {
      currentTurnPhase = PHASE_REVERSE;
      arcStartTime = now;
    }
    return;
  }

  unsigned long elapsed = now - arcStartTime;

  int crankedAngle = reversingArcServoAngle();
  setMotorOutput(BACKWARD_SPEED);
  steeringServo.write(crankedAngle);
  finalServoAngle = crankedAngle;

  float remainingAngle = abs(angleDifference);

  bool minTimeReached = elapsed >= ARC_MIN_MS;
  bool maxTimeReached = elapsed >= ARC_MAX_MS;
  bool headingClose   = remainingAngle <= ARC_EXIT_THRESHOLD;

  if ((minTimeReached && headingClose) || maxTimeReached) {
    if (maxTimeReached && !headingClose) {
      Serial.println("ARC: max time reached — exiting even though heading is still off");
    }
    finishArcTurn();
  }
}


float wrapHeading(float deg) {
  deg = fmod(deg, 360.0);
  if (deg < 0.0) deg += 360.0;
  return deg;
}

void resumeStraightDriving() {
  currentState = DRIVING_STRAIGHT;
  waypointReady = false;
  rampArmedForThisPhase = false;
  lastHeadingError = 0.0;
  lastHeadingTime = millis();
  integralError = 0.0;
}

int servoAngleFromRadius(float radiusCm) {
  int offset = 0;
  if (!isfinite(radiusCm) || fabs(radiusCm) > 400.0) {
    offset = 0;
  } else {
    float steerDeg = degrees(atan(WHEELBASE_CM / fabs(radiusCm)));
    offset = (int)round(steerDeg);
    offset = constrain(offset, 0, DIFF);
  }

  bool turnRight = isfinite(radiusCm) && radiusCm > 0.0;
  int angle;
  if (offset == 0) {
    angle = SERVO_CENTER;
  } else if (INVERT_STEERING) {
    angle = turnRight ? (SERVO_CENTER + offset) : (SERVO_CENTER - offset);
  } else {
    angle = turnRight ? (SERVO_CENTER - offset) : (SERVO_CENTER + offset);
  }
  return constrain(angle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
}

void handlePiStop() {
  lastPiCmd = millis();
  // Duplicate STOP while already holding / arcing — do not clear waypointReady
  // or restart the 400 ms pause (Pi retransmits because USB drops packets).
  if (currentState == PI_HOLD || currentState == WAYPOINT_ARC) {
    return;
  }
  waypointStartHeading = getSmoothedHeading();
  piHoldStart = millis();
  waypointReady = false;
  currentState = PI_HOLD;
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  Serial.println("PI: STOP — holding");
}

void handlePiWaypoint(String line) {
  // WAYPOINT,color,xa,ya,xc,yc,R,theta,arclen
  lastPiCmd = millis();

  String parts[9];
  int partCount = 0;
  int start = 0;
  while (partCount < 9) {
    int comma = line.indexOf(',', start);
    if (comma < 0) {
      parts[partCount++] = line.substring(start);
      break;
    }
    parts[partCount++] = line.substring(start, comma);
    start = comma + 1;
  }
  if (partCount < 9) {
    Serial.println("PI: WAYPOINT parse error");
    return;
  }

  if (currentState == WAYPOINT_ARC) {
    // Already driving to C — ignore USB retransmits.
    return;
  }

  waypointThetaDeg = parts[7].toFloat();
  waypointArcLenCm = parts[8].toFloat();
  String rStr = parts[6];
  rStr.trim();
  rStr.toLowerCase();
  if (rStr == "inf") {
    waypointRadiusCm = INFINITY;
  } else {
    waypointRadiusCm = rStr.toFloat();
  }

  if (currentState != PI_HOLD) {
    waypointStartHeading = getSmoothedHeading();
    piHoldStart = millis();
    currentState = PI_HOLD;
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);
    finalServoAngle = SERVO_CENTER;
  }

  waypointTargetHeading = wrapHeading(waypointStartHeading + waypointThetaDeg);
  waypointServoAngle = servoAngleFromRadius(waypointRadiusCm);
  waypointReady = true;

  Serial.print("PI: WAYPOINT R=");
  Serial.print(waypointRadiusCm);
  Serial.print(" theta=");
  Serial.print(waypointThetaDeg);
  Serial.print(" servo=");
  Serial.println(waypointServoAngle);
}

void handlePiReverse() {
  lastPiCmd = millis();
  waypointReady = false;
  currentState = PI_REVERSE;
  Serial.println("PI: REVERSE");
}

void handlePiClear() {
  if (currentState == WAYPOINT_ARC) {
    // Detection flicker used to send CLEAR and abort the arc mid-turn.
    return;
  }
  if (currentState == OBSTACLE_AVOIDING || currentState == PI_HOLD ||
      currentState == PI_REVERSE) {
    if (currentState == PI_HOLD) {
      straightTargetHeading = waypointStartHeading;
    }
    resumeStraightDriving();
    Serial.println("PI: CLEAR — returning to path");
  }
}

void handlePiLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  int comma = line.indexOf(',');
  String name = (comma < 0) ? line : line.substring(0, comma);
  name.trim();
  name.toUpperCase();

  if (name == "STOP") {
    handlePiStop();
  } else if (name == "WAYPOINT") {
    handlePiWaypoint(line);
  } else if (name == "REVERSE") {
    handlePiReverse();
  } else if (name == "CLEAR") {
    handlePiClear();
  } else if (name == "RED") {
    avoidDirectionRight = true;
    currentState = OBSTACLE_AVOIDING;
    lastObstacleCmd = millis();
    lastPiCmd = millis();
    Serial.println("OBSTACLE: RED - Swerving RIGHT");
  } else if (name == "GREEN") {
    avoidDirectionRight = false;
    currentState = OBSTACLE_AVOIDING;
    lastObstacleCmd = millis();
    lastPiCmd = millis();
    Serial.println("OBSTACLE: GREEN - Swerving LEFT");
  }
}

void executePiHold() {
  setMotorOutput(0);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;

  if (waypointReady && (millis() - piHoldStart >= WAYPOINT_PAUSE_MS)) {
    currentState = WAYPOINT_ARC;
    waypointArcStart = millis();
    rampArmedForThisPhase = false;
    Serial.println("PI: driving arc to C");
  } else if (!waypointReady && (millis() - piHoldStart > OBSTACLE_TIMEOUT_MS)) {
    straightTargetHeading = waypointStartHeading;
    resumeStraightDriving();
    Serial.println("PI: STOP timeout — no WAYPOINT, resuming");
  }
}

void executeWaypointArc(float currentHeading) {
  setMotorOutput(STRAIGHT_SPEED);
  steeringServo.write(waypointServoAngle);
  finalServoAngle = waypointServoAngle;

  angleDifference = shortestAngleDiff(currentHeading, waypointTargetHeading);
  unsigned long elapsed = millis() - waypointArcStart;

  bool headingClose = abs(angleDifference) <= WAYPOINT_EXIT_DEG;
  bool minTime = elapsed >= WAYPOINT_MIN_MS;
  bool maxTime = elapsed >= WAYPOINT_MAX_MS;

  unsigned long expectedMs = 1000;
  if (ESTIMATED_FWD_CMS > 1.0 && waypointArcLenCm > 0.0) {
    expectedMs = (unsigned long)(1000.0 * waypointArcLenCm / ESTIMATED_FWD_CMS);
    if (expectedMs < WAYPOINT_MIN_MS) expectedMs = WAYPOINT_MIN_MS;
    if (expectedMs > WAYPOINT_MAX_MS) expectedMs = WAYPOINT_MAX_MS;
  }
  bool distanceGuess = elapsed >= expectedMs && headingClose;

  if ((minTime && headingClose) || distanceGuess || maxTime) {
    steeringServo.write(SERVO_CENTER);
    finalServoAngle = SERVO_CENTER;
    straightTargetHeading = waypointStartHeading;
    resumeStraightDriving();
    Serial.println("PI: arrived at C — resuming original heading");
  }
}

void executePiReverse() {
  setMotorOutput(BACKWARD_SPEED);
  steeringServo.write(SERVO_CENTER);
  finalServoAngle = SERVO_CENTER;
  if (millis() - lastPiCmd > OBSTACLE_TIMEOUT_MS) {
    resumeStraightDriving();
    Serial.println("PI: REVERSE timeout — resuming");
  }
}


// ==========================================
//      USB SERIAL TELEMETRY HELPERS
// ==========================================
template <typename T>
void btPrint(T msg) {
  Serial.print(msg);
}


template <typename T>
void btPrintln(T msg) {
  Serial.println(msg);
}


void printTelemetry(float currentHeading) {
  if (currentState == DRIVING_STRAIGHT) btPrint("MODE: STRAIGHT");
  else if (currentState == TURNING) {
    if (currentTurnPhase == PHASE_PAUSE) btPrint("MODE: PAUSE   ");
    else                                 btPrint("MODE: ARC     ");
  }
  else if (currentState == OBSTACLE_AVOIDING) btPrint("MODE: AVOID   ");
  else if (currentState == PI_HOLD)           btPrint("MODE: PI-HOLD ");
  else if (currentState == WAYPOINT_ARC)      btPrint("MODE: GOTO-C  ");
  else if (currentState == PI_REVERSE)        btPrint("MODE: REVERSE ");


  btPrint(" | L: "); btPrint(currentLeftDist); btPrint("cm");
  btPrint(" | C: "); btPrint(currentCenterDist); btPrint("cm");
  btPrint(" | R: "); btPrint(currentRightDist); btPrint("cm");
  btPrint(" | Turns: "); btPrint(totalTurnsCount);


  if (hasTurnedOnce) {
    btPrint(" | LOCK: "); btPrint(lockedDirectionLeft ? "LEFT_ONLY" : "RIGHT_ONLY");
  } else {
    btPrint(" | LOCK: NONE");
  }


  if (currentState == DRIVING_STRAIGHT) {
    btPrint(" | Target: "); btPrint(straightTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Error: "); btPrint(headingError);
    btPrint("°");
  } else if (currentState == TURNING) {
    btPrint(" | Target: "); btPrint(turnTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Delta: "); btPrint(abs(angleDifference));
    btPrint("°");
    if (currentTurnPhase == PHASE_PAUSE) {
      btPrint(" | PauseMs: "); btPrint(millis() - turnPhaseStartTime);
    } else {
      btPrint(" | ArcMs: "); btPrint(millis() - arcStartTime);
    }
  } else if (currentState == WAYPOINT_ARC) {
    btPrint(" | Target: "); btPrint(waypointTargetHeading);
    btPrint("° | Current: "); btPrint(currentHeading);
    btPrint("° | Delta: "); btPrint(abs(angleDifference));
    btPrint("°");
  }


  btPrint(" | Servo Angle: "); btPrint(finalServoAngle); btPrintln("°");
}



void recomputeServoLimits() {
  SERVO_MAX_LEFT  = SERVO_CENTER + DIFF;
  SERVO_MAX_RIGHT = SERVO_CENTER - DIFF;
}

void loadTunables() {
  prefs.begin("tuning", true); // read-only
  STRAIGHT_SPEED        = prefs.getInt("straight", STRAIGHT_SPEED);
  TURN_SPEED            = prefs.getInt("turn", TURN_SPEED);
  BACKWARD_SPEED        = prefs.getInt("back", BACKWARD_SPEED);
  SERVO_CENTER          = prefs.getInt("center", SERVO_CENTER);
  DIFF                  = prefs.getInt("diff", DIFF);
  FRONT_TURN_DISTANCE   = prefs.getInt("front", FRONT_TURN_DISTANCE);
  ARC_SERVO_ANGLE       = prefs.getInt("arcang", ARC_SERVO_ANGLE);
  ARC_EXIT_THRESHOLD    = prefs.getFloat("arcexit", ARC_EXIT_THRESHOLD);
  ARC_PAUSE_MS          = prefs.getULong("arcpause", ARC_PAUSE_MS);
  ARC_MIN_MS            = prefs.getULong("arcmin", ARC_MIN_MS);
  ARC_MAX_MS            = prefs.getULong("arcmax", ARC_MAX_MS);
  prefs.end();
  recomputeServoLimits();
}

void saveTunables() {
  prefs.begin("tuning", false); // read-write
  prefs.putInt("straight", STRAIGHT_SPEED);
  prefs.putInt("turn", TURN_SPEED);
  prefs.putInt("back", BACKWARD_SPEED);
  prefs.putInt("center", SERVO_CENTER);
  prefs.putInt("diff", DIFF);
  prefs.putInt("front", FRONT_TURN_DISTANCE);
  prefs.putInt("arcang", ARC_SERVO_ANGLE);
  prefs.putFloat("arcexit", ARC_EXIT_THRESHOLD);
  prefs.putULong("arcpause", ARC_PAUSE_MS);
  prefs.putULong("arcmin", ARC_MIN_MS);
  prefs.putULong("arcmax", ARC_MAX_MS);
  prefs.end();
}

String buildValuesJson() {
  String json = "{";
  json += "\"center\":" + String(SERVO_CENTER) + ",";
  json += "\"diff\":" + String(DIFF) + ",";
  json += "\"straight\":" + String(STRAIGHT_SPEED) + ",";
  json += "\"turn\":" + String(TURN_SPEED) + ",";
  json += "\"back\":" + String(BACKWARD_SPEED) + ",";
  json += "\"front\":" + String(FRONT_TURN_DISTANCE) + ",";
  json += "\"arcang\":" + String(ARC_SERVO_ANGLE) + ",";
  json += "\"arcexit\":" + String(ARC_EXIT_THRESHOLD, 1) + ",";
  json += "\"arcpause\":" + String(ARC_PAUSE_MS) + ",";
  json += "\"arcmin\":" + String(ARC_MIN_MS) + ",";
  json += "\"arcmax\":" + String(ARC_MAX_MS) + ",";
  json += "\"maxLeft\":" + String(SERVO_MAX_LEFT) + ",";
  json += "\"maxRight\":" + String(SERVO_MAX_RIGHT);
  json += "}";
  return json;
}

void handleGetValues() {
  server.send(200, "application/json", buildValuesJson());
}

void handleSetValues() {
  if (server.hasArg("center"))   SERVO_CENTER          = server.arg("center").toInt();
  if (server.hasArg("diff"))     DIFF                  = server.arg("diff").toInt();
  if (server.hasArg("straight")) STRAIGHT_SPEED        = server.arg("straight").toInt();
  if (server.hasArg("turn"))     TURN_SPEED            = server.arg("turn").toInt();
  if (server.hasArg("back"))     BACKWARD_SPEED        = server.arg("back").toInt();
  if (server.hasArg("front"))    FRONT_TURN_DISTANCE   = server.arg("front").toInt();
  if (server.hasArg("arcang"))   ARC_SERVO_ANGLE       = server.arg("arcang").toInt();
  if (server.hasArg("arcexit"))  ARC_EXIT_THRESHOLD    = server.arg("arcexit").toFloat();
  if (server.hasArg("arcpause")) ARC_PAUSE_MS          = server.arg("arcpause").toInt();
  if (server.hasArg("arcmin"))   ARC_MIN_MS            = server.arg("arcmin").toInt();
  if (server.hasArg("arcmax"))   ARC_MAX_MS            = server.arg("arcmax").toInt();

  recomputeServoLimits();
  saveTunables(); // so it survives a reboot/power cycle

  server.send(200, "application/json", buildValuesJson());
}

// The tuning page itself, served directly by the ESP32 — no internet required.
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robot Tuner</title>
<style>
  body{ background:#0d1113; color:#e8edee; font-family:-apple-system,Arial,sans-serif; margin:0; padding:20px; }
  h1{ font-size:20px; margin:0 0 4px; }
  .sub{ color:#8fa0a5; font-size:12.5px; margin-bottom:20px; }
  .field{ margin-bottom:22px; }
  .row{ display:flex; justify-content:space-between; margin-bottom:6px; }
  .name{ font-size:14px; }
  .val{ font-family:monospace; color:#ffb020; font-weight:bold; }
  input[type=range]{ width:100%; accent-color:#ffb020; }
  #status{ font-family:monospace; font-size:12px; color:#57d9c9; margin-top:10px; min-height:18px; }
  #status.err{ color:#ff6b5e; }
</style>
</head>
<body>
  <h1>Robot Tuner</h1>
  <div class="sub">Connected directly to the robot's WiFi. Changes apply instantly.</div>

  <div class="field">
    <div class="row"><span class="name">SERVO_CENTER</span><span class="val" id="v-center">--</span></div>
    <input type="range" id="s-center" min="0" max="180" value="117">
  </div>
  <div class="field">
    <div class="row"><span class="name">DIFF</span><span class="val" id="v-diff">--</span></div>
    <input type="range" id="s-diff" min="5" max="90" value="25">
  </div>
  <div class="field">
    <div class="row"><span class="name">STRAIGHT_SPEED</span><span class="val" id="v-straight">--</span></div>
    <input type="range" id="s-straight" min="0" max="255" value="80">
  </div>
  <div class="field">
    <div class="row"><span class="name">TURN_SPEED</span><span class="val" id="v-turn">--</span></div>
    <input type="range" id="s-turn" min="0" max="255" value="80">
  </div>
  <div class="field">
    <div class="row"><span class="name">BACKWARD_SPEED</span><span class="val" id="v-back">--</span></div>
    <input type="range" id="s-back" min="-255" max="0" value="-80">
  </div>
  <div class="field">
    <div class="row"><span class="name">FRONT_TURN_DISTANCE (cm)</span><span class="val" id="v-front">--</span></div>
    <input type="range" id="s-front" min="10" max="150" value="50">
  </div>
  <div class="field">
    <div class="row"><span class="name">ARC_SERVO_ANGLE</span><span class="val" id="v-arcang">--</span></div>
    <input type="range" id="s-arcang" min="5" max="90" value="20">
  </div>
  <div class="field">
    <div class="row"><span class="name">ARC_EXIT_THRESHOLD</span><span class="val" id="v-arcexit">--</span></div>
    <input type="range" id="s-arcexit" min="2" max="30" value="8">
  </div>
  <div class="field">
    <div class="row"><span class="name">ARC_PAUSE_MS</span><span class="val" id="v-arcpause">--</span></div>
    <input type="range" id="s-arcpause" min="0" max="2000" value="500">
  </div>
  <div class="field">
    <div class="row"><span class="name">ARC_MIN_MS</span><span class="val" id="v-arcmin">--</span></div>
    <input type="range" id="s-arcmin" min="0" max="2000" value="400">
  </div>
  <div class="field">
    <div class="row"><span class="name">ARC_MAX_MS</span><span class="val" id="v-arcmax">--</span></div>
    <input type="range" id="s-arcmax" min="500" max="8000" value="4000">
  </div>

  <div id="status">Loading current values...</div>

<script>
  const ids = ['center','diff','straight','turn','back','front','arcang','arcexit','arcpause','arcmin','arcmax'];
  const sliders = {}; const labels = {};
  ids.forEach(id => {
    sliders[id] = document.getElementById('s-' + id);
    labels[id]  = document.getElementById('v-' + id);
  });

  const statusEl = document.getElementById('status');
  let sendTimer = null;

  function paintLabels(){
    labels.center.textContent = sliders.center.value;
    labels.diff.textContent = sliders.diff.value;
    labels.straight.textContent = sliders.straight.value;
    labels.turn.textContent = sliders.turn.value;
    labels.back.textContent = sliders.back.value;
    labels.front.textContent = sliders.front.value;
    labels.arcang.textContent = sliders.arcang.value;
    labels.arcexit.textContent = parseFloat(sliders.arcexit.value).toFixed(1);
    labels.arcpause.textContent = sliders.arcpause.value;
    labels.arcmin.textContent = sliders.arcmin.value;
    labels.arcmax.textContent = sliders.arcmax.value;
  }

  function loadFromRobot(){
    fetch('/get').then(r => r.json()).then(d => {
      sliders.center.value = d.center;
      sliders.diff.value = d.diff;
      sliders.straight.value = d.straight;
      sliders.turn.value = d.turn;
      sliders.back.value = d.back;
      sliders.front.value = d.front;
      sliders.arcang.value = d.arcang;
      sliders.arcexit.value = d.arcexit;
      sliders.arcpause.value = d.arcpause;
      sliders.arcmin.value = d.arcmin;
      sliders.arcmax.value = d.arcmax;
      paintLabels();
      statusEl.textContent = 'Synced with robot.';
      statusEl.className = '';
    }).catch(() => {
      statusEl.textContent = 'Could not reach robot.';
      statusEl.className = 'err';
    });
  }

  function sendToRobot(){
    const q = ids.map(id => id + '=' + sliders[id].value).join('&');
    fetch('/set?' + q).then(r => r.json()).then(() => {
      statusEl.textContent = 'Applied on robot.';
      statusEl.className = '';
    }).catch(() => {
      statusEl.textContent = 'Robot unreachable — value not sent.';
      statusEl.className = 'err';
    });
  }

  ids.forEach(id => {
    sliders[id].addEventListener('input', () => {
      paintLabels();
      statusEl.textContent = 'Sending...';
      statusEl.className = '';
      clearTimeout(sendTimer);
      sendTimer = setTimeout(sendToRobot, 200);
    });
  });

  loadFromRobot();
</script>
</body>
</html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void setupWifiTuner() {
  loadTunables(); // pull any previously-saved values before anything else uses them

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(200);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("Tuner WiFi: "); Serial.println(AP_SSID);
  Serial.print("Tuner IP:   "); Serial.println(ip);

  if (MDNS.begin("robot")) {
    Serial.println("mDNS ready — try http://robot.local");
  }

  server.on("/", handleRoot);
  server.on("/get", handleGetValues);
  server.on("/set", handleSetValues);
  server.begin();
}


// ==========================================
//             CORE ARDUINO SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  // setupWifiTuner(); // loads saved tuning values (or the defaults above) + starts the web page

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);


  selectMuxChannel(MUX_CH_BNO);
  if (!bno.begin()) {
    Serial.println("Critical Error: BNO055 missing on Multiplexer channel 4!");
    while (1) { server.handleClient(); } // keep the tuner alive even if the BNO fails
  }
  delay(500);
  bno.setExtCrystalUse(true);

  // Wait for the gyro to calibrate before trusting the heading reading
  Serial.println("Waiting for BNO055 calibration...");
  uint8_t sysCal, gyroCal, accelCal, magCal;
  unsigned long calStart = millis();
  do {
    selectMuxChannel(MUX_CH_BNO);              // must re-select the mux channel every I2C call
    bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    Serial.print("Cal - Sys:"); Serial.print(sysCal);
    Serial.print(" Gyro:"); Serial.print(gyroCal);
    Serial.print(" Accel:"); Serial.print(accelCal);
    Serial.print(" Mag:"); Serial.println(magCal);
    delay(200);
  } while (gyroCal < 3 && millis() - calStart < 10000);  // wait up to 10s, prioritize gyro
  Serial.println("Calibration wait done.");

  straightTargetHeading = getCurrentHeading();

  // Begin the speed ramp for the very first start from a stand-still
  driveStartTime = millis();
  rampActive = true;
}


void avoidObstacle() {
  // Keep moving forward (use straight speed so the robot doesn't stop)
  setMotorOutput(STRAIGHT_SPEED);

  // Choose swerve angle based on direction and steering inversion
  int swerveAngle;
  if (avoidDirectionRight) {
    swerveAngle = INVERT_STEERING ? SERVO_MAX_LEFT : SERVO_MAX_RIGHT;
  } else {
    swerveAngle = INVERT_STEERING ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT;
  }

  steeringServo.write(swerveAngle);
  finalServoAngle = swerveAngle;
}

// ==========================================
//             MAIN EXECUTION LOOP
// ==========================================
void loop() {
  server.handleClient(); // always service the tuning page, in every state
  // ---- Serial command parser (Pi: STOP / WAYPOINT / REVERSE / CLEAR, plus RED/GREEN) ----
  static String serialBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handlePiLine(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  // ---- Pi / obstacle safety timeout ----
  if (currentState == OBSTACLE_AVOIDING && (millis() - lastObstacleCmd > OBSTACLE_TIMEOUT_MS)) {
    resumeStraightDriving();
    Serial.println("OBSTACLE: Timeout - Auto returning to path");
  }
  if (currentState == ROBOT_STOPPED) {
    setMotorOutput(0);
    steeringServo.write(SERVO_CENTER);

    Serial.print("STATUS: Finished. Total Turns Executed: ");
    Serial.println(totalTurnsCount);

    delay(200);
    return;
  }


  currentLeftDist   = getLunaDistance(MUX_CH_LEFT);
  currentCenterDist = getLunaDistance(MUX_CH_CENTER);
  currentRightDist  = getLunaDistance(MUX_CH_RIGHT);


  float currentHeading = getSmoothedHeading();


  // Stop condition: left AND right both under 100cm, and front (center) under 150cm.
  // Must hold continuously for STOP_CONFIRM_MS before actually stopping,
  // to avoid triggering on a single noisy/transient reading.
  bool stopConditionMet =
      (currentLeftDist   > 0 && currentLeftDist   < 100) &&
      (currentRightDist  > 0 && currentRightDist  < 100) &&
      (currentCenterDist > 0 && currentCenterDist < 150) &&
      (totalTurnsCount >= MAX_TURNS);

  if (stopConditionMet) {
    if (!stopConditionActive) {
      stopConditionActive = true;
      stopConditionStartTime = millis();
    } else if (millis() - stopConditionStartTime >= STOP_CONFIRM_MS) {
      currentState = ROBOT_STOPPED;
      return;
    }
  } else {
    stopConditionActive = false;
  }


  if (currentState == DRIVING_STRAIGHT) {
    checkFrontObstacle();
  }

  if (currentState == DRIVING_STRAIGHT) {
    driveStraightMode(currentHeading);
  }
  else if (currentState == TURNING) {
    executeTurnMode(currentHeading);
  }
  else if (currentState == OBSTACLE_AVOIDING) {
    avoidObstacle();
  }
  else if (currentState == PI_HOLD) {
    executePiHold();
  }
  else if (currentState == WAYPOINT_ARC) {
    executeWaypointArc(currentHeading);
  }
  else if (currentState == PI_REVERSE) {
    executePiReverse();
  }


  printTelemetry(currentHeading);
  delay(20);
}
