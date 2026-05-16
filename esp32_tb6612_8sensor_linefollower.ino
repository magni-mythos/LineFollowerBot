/*
 * ============================================================
 *  COMPETITION ESP32 LINE FOLLOWER
 *  TB6612FNG + QTR-8RC (8 sensors) — PID with Advanced Features
 *  Chassis: 180 mm × 100 mm
 *  Pin mapping: per user's established config
 * ============================================================
 */

// ─────────────────────────────────────────────────────────────────
//  TB6612FNG MOTOR DRIVER PINS  (user mapping)
//    Motor A = LEFT,  Motor B = RIGHT
// ─────────────────────────────────────────────────────────────────
#define STBY  4

// Left Motor (Motor A)
#define AIN1  21
#define AIN2  22
#define PWMA  23

// Right Motor (Motor B)
#define BIN1  19
#define BIN2  18
#define PWMB  5

// ─────────────────────────────────────────────────────────────────
//  LEDC PWM (ESP32)
// ─────────────────────────────────────────────────────────────────
#define PWM_CHANNEL_L    0       // Motor A (LEFT)
#define PWM_CHANNEL_R    1       // Motor B (RIGHT)
#define PWM_FREQ     20000       // 20 kHz — inaudible
#define PWM_RESOLUTION   8       // 0–255

// ─────────────────────────────────────────────────────────────────
//  QTR-8RC IR EMITTER CONTROL (LEDON pin)
//  Tie LEDON to this GPIO for pulsed-LED reads.
//  If LEDON is wired straight to VCC, set USE_EMITTER_CTRL = false.
// ─────────────────────────────────────────────────────────────────
#define EMITTER_PIN          15
#define USE_EMITTER_CTRL  true

// ─────────────────────────────────────────────────────────────────
//  8 QTR-8RC SENSOR PINS  (user mapping, S0 = leftmost)
// ─────────────────────────────────────────────────────────────────
const uint8_t IR_PINS[8] = {
  32,  // S0 — leftmost
  33,  // S1
  25,  // S2
  26,  // S3
  27,  // S4
  14,  // S5
  12,  // S6
  13   // S7 — rightmost
};

// ─────────────────────────────────────────────────────────────────
//  QTR-8RC TIMING
// ─────────────────────────────────────────────────────────────────
#define QTR_CHARGE_US       10
#define QTR_TIMEOUT_US    2500
#define QTR_THRESHOLD_US  1200    // > this = sensor sees BLACK

// ─────────────────────────────────────────────────────────────────
//  SPEED & PID TUNING
// ─────────────────────────────────────────────────────────────────
int   BASE_SPEED =  180;
int   MAX_SPEED  =  255;
int   MIN_SPEED  = -100;     // negative → inner wheel reverses on tight turns
int   TURN_SPEED =  120;

float Kp = 45.0;
float Ki =  0.0;
float Kd = 65.0;

const float DERIVATIVE_FILTER_ALPHA = 0.7;
const int   MAX_ACCEL               = 15;

// ─────────────────────────────────────────────────────────────────
//  SENSOR WEIGHTS  — symmetric around centreline, range ±3.5
// ─────────────────────────────────────────────────────────────────
const float SENSOR_WEIGHTS[8] = {
  -3.5f, -2.5f, -1.5f, -0.5f,
  +0.5f, +1.5f, +2.5f, +3.5f
};
const float POSITION_MAX = 3.5f;

// ─────────────────────────────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────────────────────────────
float    previousError      = 0;
float    integral           = 0;
float    filteredDerivative = 0;
int      lastDirection      = 1;     // 1 = right, −1 = left
uint8_t  sensorBinary[8];            // 1 = sees BLACK
uint16_t sensorRaw[8];               // raw discharge time in µs
int      prevRightSpeed     = 0;
int      prevLeftSpeed      = 0;
unsigned long prevLoopTime  = 0;

// ═════════════════════════════════════════════════════════════════
//  MOTOR CONTROL
//  TB6612FNG truth table:
//    IN1=H, IN2=L          → Forward
//    IN1=L, IN2=H          → Reverse
//    IN1=L, IN2=L          → Coast
//    IN1=H, IN2=H + PWM=255→ Short brake
// ═════════════════════════════════════════════════════════════════
void setLeftMotor(int speed) {           // Motor A
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  ledcWrite(PWM_CHANNEL_L, speed);
}

void setRightMotor(int speed) {          // Motor B
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  ledcWrite(PWM_CHANNEL_R, speed);
}

void stopMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  ledcWrite(PWM_CHANNEL_L, 255);
  ledcWrite(PWM_CHANNEL_R, 255);
}

void enableDriver()  { digitalWrite(STBY, HIGH); }
void disableDriver() { digitalWrite(STBY, LOW);  }

// ═════════════════════════════════════════════════════════════════
//  ACCELERATION RAMP
// ═════════════════════════════════════════════════════════════════
int rampSpeed(int current, int target, int maxStep) {
  int diff = target - current;
  if (diff >  maxStep) return current + maxStep;
  if (diff < -maxStep) return current - maxStep;
  return target;
}

// ═════════════════════════════════════════════════════════════════
//  QTR-8RC PARALLEL READ
// ═════════════════════════════════════════════════════════════════
int readSensors() {
  if (USE_EMITTER_CTRL) {
    digitalWrite(EMITTER_PIN, HIGH);
    delayMicroseconds(200);
  }

  for (int i = 0; i < 8; i++) {
    pinMode(IR_PINS[i], OUTPUT);
    digitalWrite(IR_PINS[i], HIGH);
  }
  delayMicroseconds(QTR_CHARGE_US);

  for (int i = 0; i < 8; i++) pinMode(IR_PINS[i], INPUT);

  for (int i = 0; i < 8; i++) sensorRaw[i] = QTR_TIMEOUT_US;
  unsigned long startUs   = micros();
  unsigned long elapsed   = 0;
  uint8_t       pendingMask = 0xFF;

  while (pendingMask && elapsed < QTR_TIMEOUT_US) {
    elapsed = micros() - startUs;
    for (int i = 0; i < 8; i++) {
      if (pendingMask & (1 << i)) {
        if (digitalRead(IR_PINS[i]) == LOW) {
          sensorRaw[i] = (uint16_t)elapsed;
          pendingMask &= ~(1 << i);
        }
      }
    }
  }

  if (USE_EMITTER_CTRL) digitalWrite(EMITTER_PIN, LOW);

  int activeCount = 0;
  for (int i = 0; i < 8; i++) {
    sensorBinary[i] = (sensorRaw[i] > QTR_THRESHOLD_US) ? 1 : 0;
    if (sensorBinary[i]) activeCount++;
  }
  return activeCount;
}

// ═════════════════════════════════════════════════════════════════
//  WEIGHTED CENTROID  →  position in [−3.5, +3.5]
// ═════════════════════════════════════════════════════════════════
float computeLinePosition(bool &lineDetected) {
  float weightedSum = 0;
  float totalActive = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorBinary[i]) {
      weightedSum += SENSOR_WEIGHTS[i];
      totalActive += 1.0f;
    }
  }

  if (totalActive == 0) {
    lineDetected = false;
    return 0;
  }
  lineDetected = true;
  return weightedSum / totalActive;
}

// ═════════════════════════════════════════════════════════════════
//  TIME-BASED PID + DERIVATIVE LOW-PASS FILTER
// ═════════════════════════════════════════════════════════════════
float computePID(float error, float dt) {
  float P = Kp * error;

  integral += error * dt;
  integral  = constrain(integral, -50.0f, 50.0f);
  float I   = Ki * integral;

  float rawDerivative = (dt > 0) ? (error - previousError) / dt : 0;
  filteredDerivative  = DERIVATIVE_FILTER_ALPHA * filteredDerivative
                      + (1.0f - DERIVATIVE_FILTER_ALPHA) * rawDerivative;
  float D = Kd * filteredDerivative;

  previousError = error;
  return P + I + D;
}

// ═════════════════════════════════════════════════════════════════
//  SERIAL TUNING:  P<val>  I<val>  D<val>  S<base>  T<turn>
// ═════════════════════════════════════════════════════════════════
void handleSerialTuning() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() < 2) return;

  char  type = cmd.charAt(0);
  float val  = cmd.substring(1).toFloat();

  switch (type) {
    case 'P': case 'p': Kp = val; break;
    case 'I': case 'i': Ki = val; break;
    case 'D': case 'd': Kd = val; break;
    case 'S': case 's': BASE_SPEED = (int)val; break;
    case 'T': case 't': TURN_SPEED = (int)val; break;
    default:
      Serial.println("Cmds: P<val> I<val> D<val> S<base> T<turn>");
      return;
  }
  Serial.printf("Set %c = %.3f\n", type, val);
}

// ═════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 Line Follower — TB6612FNG + QTR-8RC");
  Serial.println("Chassis 180 x 100 mm, sensor span ~66.7 mm");

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);

  if (USE_EMITTER_CTRL) {
    pinMode(EMITTER_PIN, OUTPUT);
    digitalWrite(EMITTER_PIN, LOW);
  }

  for (int i = 0; i < 8; i++) pinMode(IR_PINS[i], INPUT);

  enableDriver();
  prevLoopTime = micros();

  delay(1000);
  Serial.println("3..."); delay(1000);
  Serial.println("2..."); delay(1000);
  Serial.println("1..."); delay(1000);
  Serial.println("GO!");
}

// ═════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = micros();
  float dt = (now - prevLoopTime) / 1000000.0f;
  prevLoopTime = now;

  handleSerialTuning();

  int activeSensors = readSensors();
  bool allWhite = (activeSensors == 0);
  bool allBlack = (activeSensors >= 7);

  if (allWhite) {
    if (lastDirection < 0) { setRightMotor( TURN_SPEED); setLeftMotor(-TURN_SPEED); }
    else                   { setRightMotor(-TURN_SPEED); setLeftMotor( TURN_SPEED); }
    integral = 0;
    filteredDerivative = 0;
    return;
  }

  if (allBlack) {
    setRightMotor(BASE_SPEED);
    setLeftMotor(BASE_SPEED);
    return;
  }

  bool  lineDetected = false;
  float linePosition = computeLinePosition(lineDetected);
  float error        = linePosition;

  if      (error < -0.3f) lastDirection = -1;
  else if (error >  0.3f) lastDirection =  1;

  float correction = computePID(error, dt);

  float turnRatio = fabsf(error) / POSITION_MAX;
  turnRatio       = turnRatio * turnRatio;
  int dynamicBase = BASE_SPEED - (int)(turnRatio * (BASE_SPEED - TURN_SPEED));

  int targetRight = constrain(dynamicBase - (int)correction, MIN_SPEED, MAX_SPEED);
  int targetLeft  = constrain(dynamicBase + (int)correction, MIN_SPEED, MAX_SPEED);

  int rightSpeed = rampSpeed(prevRightSpeed, targetRight, MAX_ACCEL);
  int leftSpeed  = rampSpeed(prevLeftSpeed,  targetLeft,  MAX_ACCEL);
  prevRightSpeed = rightSpeed;
  prevLeftSpeed  = leftSpeed;

  setRightMotor(rightSpeed);
  setLeftMotor(leftSpeed);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf("Pos:%+5.2f Err:%+5.2f Cor:%+5.0f R:%4d L:%4d dt:%4.1fms act:%d\n",
                  linePosition, error, correction,
                  rightSpeed, leftSpeed, dt * 1000.0f, activeSensors);
  }
}
