// Define the specific pin where you connected the long leg of your LED
int ledPin = 2; // If you used a different pin, just change this number

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);
  delay(2000); // Give the monitor time to open
  
  Serial.println("--- ESP32 External LED Test Started ---");

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Turn the LED on
  digitalWrite(ledPin, HIGH);
  Serial.println("Status: ON");
  delay(1000); // Wait for 1 second

  // Turn the LED off
  digitalWrite(ledPin, LOW);
  Serial.println("Status: OFF");
  delay(1000); // Wait for 1 second
}