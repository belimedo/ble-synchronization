#define LED_PIN 23  // GPIO 23

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Set GPIO 23 as output
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // Turn LED ON
  delay(1000);                  // Wait 1 second
  digitalWrite(LED_PIN, LOW);   // Turn LED OFF
  delay(1000);                  // Wait 1 second
}