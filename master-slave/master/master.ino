#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

// UUIDs (must match your slave device)
#define SERVICE_UUID        "4FAFC201-1FB5-459E-8FCC-C5C9C331914B"
#define DATA_CHAR_UUID      "A9B1C2D3-E4F5-1234-5678-9ABC12345678"
#define LED_PIN 2  // Onboard LED (GPIO2)

BLEClient* pClient;
bool dataReceived = false;
uint16_t receivedBuffer[100];

// Notification callback (correct signature for v1.3.7)
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  if (length == sizeof(receivedBuffer)) {
    memcpy(receivedBuffer, pData, length);
    dataReceived = true;
    Serial.println("Received complete buffer");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("HIGH LED");
  delay(100);
  
  // Initialize BLE (correct for v1.3.7)
  BLEDevice::init("ESP32_Master");
  pClient = BLEDevice::createClient();
  
  // Scan for devices
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = *pBLEScan->start(5); // 5 second scan

  // Connect to first device with our service
  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    if (device.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      pClient->connect(&device);
      break;
    }
  }

  // Setup notifications
  if (pClient->isConnected()) {
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService != nullptr) {
      BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(DATA_CHAR_UUID);
      if (pRemoteCharacteristic != nullptr && pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println("Registered for notifications");
      }
    }
  }
}

void loop() {
  Serial.println("HIGH LED");
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  Serial.println("LOW LED");
  digitalWrite(LED_PIN, LOW);
  delay(100);
  if (dataReceived) {
    Serial.println("Processing buffer:");
    for (int i = 0; i < 100; i++) {
      Serial.printf("Value %d: %d -> ", i, receivedBuffer[i]);
      if (receivedBuffer[i] > 100) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ON");
      } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED OFF");
      }
      delay(200);
    }
    dataReceived = false;
  }
}