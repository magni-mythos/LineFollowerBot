#include <SparkFun_TB6612.h>

// --- Hardware Pin Configuration ---
// These match your exact ESP32 wiring
#define STBY 4
#define AIN1 21
#define AIN2 22
#define PWMA 23
#define BIN1 19
#define BIN2 18
#define PWMB 5

// Motor offsets (change to -1 if a motor spins backwards when it should go forward)
const int offsetA = 1;
const int offsetB = 1;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Left Motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Right Motor

void setup() {
  Serial.begin(115200);
  delay(2000); // Give the Serial Monitor time to open
  Serial.println("--- TB6612FNG Hardware Test Starting ---");
}

void loop() {
  Serial.println("Moving Forward...");
  // .drive() takes a speed from -255 to 255. 150 is a safe testing speed.
  motor1.drive(150); 
  motor2.drive(150);
  delay(2000);       // Run for 2 seconds

  Serial.println("Stopping...");
  motor1.brake();
  motor2.brake();
  delay(1000);       // Stop for 1 second

  Serial.println("Moving Backward...");
  motor1.drive(-150); // Negative numbers make it spin in reverse
  motor2.drive(-150);
  delay(2000);       // Run for 2 seconds

  Serial.println("Stopping...");
  motor1.brake();
  motor2.brake();
  delay(2000);       // Stop for 2 seconds before repeating the whole loop
}