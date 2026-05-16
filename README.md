# Autonomous Line Follower Robot

A robust, high-speed autonomous line-following robot designed to navigate highly complex track layouts. Built by **Team Oblivion**, this system is engineered to handle massive circular intersections, 90-degree switchbacks, and multi-path "trident" junctions where standard PID control typically fails.

## ⚙️ How It Works (Control Strategy)
To conquer tracks designed to confuse basic sensors with thick intersections and false paths, we engineered three core software mechanics:

* **High-Gain, Low-Speed Tuning:** Instead of relying on pure forward momentum, we used a highly tuned, aggressive PD loop. A low base speed reduces forward inertia, while a massive turn force (`Kp`) and heavy shock absorbers (`Kd`) allow for instantaneous, stable corrections without straightaway wobbles.
* **Asymmetric Reverse Boosters (Drift):** By splitting our minimum motor speeds into asymmetrical negative limits, the inner wheel drops into reverse during sharp turns. This breaks traction and forces the robot to execute aggressive, smooth "drifts" around 90-degree switchbacks.
* **"Forced Right" Maze Solver:** To beat massive circular intersections and 3-line splits, we hardcoded a **Right-Hand Wall Follower** algorithm. When both sensors hit a massive black patch simultaneously, a logic override forces a mechanical blind pivot to lock onto the outermost right path.

## 🛠️ Hardware Specifications
This chassis and control architecture is designed to be highly adaptable:
* **Microcontroller:** ESP32 
* **Motor Driver:** SparkFun TB6612FNG
* **Sensors (Current Setup):** 2x Digital IR Proximity Sensors, configured with a precise 40mm physical gap to perfectly straddle 3.6cm thick competition lines.
* **Sensors (Supported Upgrades):** Fully compatible with QTR-8RC (8-Channel RC Reflectance Sensor Array) for advanced, high-resolution PID line mapping.
* **Chassis:** Custom 2-wheel drive setup with DC gear motors. 

## 💻 Tech Stack
* **Language:** C++
* **Environment:** Arduino IDE
* **Core Concepts:** Embedded Systems, Proportional-Derivative (PD) Control, Hardware-Software Integration.

## 📄 Main Code Implementation (2-Sensor IR)

```cpp
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

float Kp = 145.00;  // High turning force
float Kd = 110.00;  // Massive shock absorbers to prevent straightaway wobble 

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
    error = 0; // Straddling perfectly
  } 
  else if (leftVal == HIGH && rightVal == LOW) {
    error = -1; // Drifted right
  } 
  else if (leftVal == LOW && rightVal == HIGH) {
    error = 1; // Drifted left
  } 
  else if (leftVal == HIGH && rightVal == HIGH) {
    // THE OVERRIDE: 
    // Force a hard right bend to survive massive intersections!
    error = 1; 
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
  // Asymmetric negative numbers allow aggressive tank-pivots
  int leftMinSpeed = -35;  
  int rightMinSpeed = -65; 

  leftMotorSpeed = constrain(leftMotorSpeed, leftMinSpeed, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, rightMinSpeed, 255);

  // --- DRIVE ---
  motor1.drive(leftMotorSpeed);
  motor2.drive(rightMotorSpeed);
}
```

## 🚀 Future Roadmap
* Transition from discrete IRs to a high-density **QTR-8RC sensor array**/**8 Channel IR Sensor Array** for smoother error gradients.
* Integrate a line-loss timer for automatic precision braking when the track ends.
