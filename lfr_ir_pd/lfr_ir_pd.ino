#include <SparkFun_TB6612.h>

// ==========================================
// 1. HARDWARE PINOUTS
// ==========================================
#define STBY 4
#define AIN1 21
#define AIN2 22
#define PWMA 23
#define BIN1 19
#define BIN2 18
#define PWMB 5

const int offsetA = 1; 
const int offsetB = 1; 

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Left Motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Right Motor

#define LEFT_SENSOR 13
#define RIGHT_SENSOR 14

// ==========================================
// 2. THE "HIGH-GAIN, LOW-SPEED" TUNE
// ==========================================
const int baseSpeed = 80; // Slow, methodical forward momentum

// High Kp keeps the turning violently fast.
// Massive Kd stops it from shaking itself to pieces on the straightaways.
float Kp = 145.00;  
float Kd = 110.00;   

int lastError = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  delay(3000); 
}

void loop() {
  int leftVal = digitalRead(LEFT_SENSOR);
  int rightVal = digitalRead(RIGHT_SENSOR);

  int error = 0;

  // --- ERROR MAPPING ---
  if (leftVal == LOW && rightVal == LOW) {
    error = 0;
  } 
  else if (leftVal == HIGH && rightVal == LOW) {
    error = -1; // Drifted right
  } 
  else if (leftVal == LOW && rightVal == HIGH) {
    error = 1; // Drifted left
  } 
  else if (leftVal == HIGH && rightVal == HIGH) {
    // --- THE TRIDENT & CIRCLE OVERRIDE ---
    // Because baseSpeed is 80 and Kp is 145, this error=1 will force the 
    // left wheel to blast to 225 speed, throwing it hard into the right path!
    error = 0; 
  }

  // --- PD MATH ---
  float P = error;
  float D = error - lastError;
  float correction = (Kp * P) + (Kd * D);

  lastError = error;

  // --- APPLY TO MOTORS ---
  int leftMotorSpeed = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  // --- THE DRIFT LIMITERS ---
  // The negative numbers are still here to allow the aggressive tank-pivots
  int leftMinSpeed = -35;  
  int rightMinSpeed = -65; 

  leftMotorSpeed = constrain(leftMotorSpeed, leftMinSpeed, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, rightMinSpeed, 255);

  // --- DRIVE ---
  motor1.drive(leftMotorSpeed);
  motor2.drive(rightMotorSpeed);
}