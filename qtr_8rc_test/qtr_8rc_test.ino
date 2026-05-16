#include <QTRSensors.h>

QTRSensors qtr;

// --- SENSOR SETUP ---
// Full 8-sensor array restored
const uint8_t SensorCount = 8; 
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(115200);
  delay(2000); // Give the serial monitor time to open

  qtr.setTypeRC();
  
  // All 8 pins mapped in order from Left (D1) to Right (D8)
  // D1=32, D2=33, D3=25, D4=26, D5=27, D6=14, D7=12, D8=13
  qtr.setSensorPins((const uint8_t[]){32, 33, 25, 26, 27, 14, 12, 13}, SensorCount);
  
  // NOTICE: qtr.setEmitterPin() is completely deleted. 
  // The library assumes the IR LEDs are permanently hardwired to power.

  Serial.println("--- Full 8-Sensor Hardware Bypass Diagnostic Started ---");
}

void loop() {
  // Read raw, uncalibrated data directly from the hardware
  qtr.read(sensorValues);

  // Print all 8 numbers in a row across the screen
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); 
  }
  
  Serial.println(); 
  
  // Read 4 times a second so the numbers don't blur
  delay(250);       
}