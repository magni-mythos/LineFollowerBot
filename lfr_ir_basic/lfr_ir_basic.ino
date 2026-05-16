#include <SparkFun_TB6612.h>

// --- Motor Driver Pins (Your exact original setup) ---
#define STBY 4
#define AIN1 21
#define AIN2 22
#define PWMA 23
#define BIN1 19
#define BIN2 18
#define PWMB 5

const int offsetA = 1; // Change to -1 if left motor spins backward
const int offsetB = 1; // Change to -1 if right motor spins backward

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Left Motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Right Motor

// --- NEW IR Sensor Pins ---
#define LEFT_SENSOR 13
#define RIGHT_SENSOR 14

// --- Speed Settings ---
const int driveSpeed = 120; // Speed when driving straight
const int turnSpeed = 150;  // Speed when turning (can be slightly slower for grip)

void setup() {
  Serial.begin(115200);
  
  // Set your new sensor pins to read mode
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  // 3-second pause before takeoff so you can put it on the track
  delay(3000); 
}

void loop() {
  // Read the sensors. 
  // NOTE: Most cheap IR modules output HIGH (1) for Black, and LOW (0) for White.
  int leftVal = digitalRead(LEFT_SENSOR);
  int rightVal = digitalRead(RIGHT_SENSOR);

  // LOGIC GATE 1: Both sensors see White (Line is safely in the middle)
  // LOGIC GATE 1: Both sensors see White (Line is safely in the middle)
  if (leftVal == LOW && rightVal == LOW) {
    motor1.drive(driveSpeed); 
    motor2.drive(driveSpeed); 
  } 
  
  // LOGIC GATE 2: Left sensor hits Black (Turn Left)
  else if (leftVal == HIGH && rightVal == LOW) {
    motor1.drive(-turnSpeed); // Left motor pulls backward
    motor2.drive(turnSpeed);  // Right motor pushes forward (TANK PIVOT)
  } 
  
  // LOGIC GATE 3: Right sensor hits Black (Turn Right)
  else if (leftVal == LOW && rightVal == HIGH) {
    motor1.drive(turnSpeed);   // Left motor pushes forward (TANK PIVOT)
    motor2.drive(-turnSpeed);  // Right motor pulls backward
  } 
  
  // LOGIC GATE 4: Both sensors hit Black (Crossroads or picked up)
  else {
    motor1.brake();
    motor2.brake();
  }
}