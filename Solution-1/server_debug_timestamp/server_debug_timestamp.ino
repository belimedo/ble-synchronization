#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2901.h>  // For characteristic descriptions
#include <BLE2902.h>  // For notifications
#include "esp_timer.h"

// BLE Service & Characteristics UUIDs (randomly generated)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_TIMESTAMP_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_COUNTER_UUID   "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c"

// GPIO for debug LED (onboard LED on ESP32-DevKitC)
const int debugPin = 2;
const int togglePin = 22;

// Global variables
BLEServer *pServer;
BLECharacteristic *pCharTimestamp; // Karakteristika je kontejner za podatke u sustini
BLECharacteristic *pCharCounter;
uint32_t counter = 0;
bool deviceConnected = false;
uint64_t lastTimestamp = 0;

// BLE Server Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
        pServer->startAdvertising(); // Restart advertising
    }
};

// Counter Characteristic Callback (when client reads it)
class CounterCharCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* pCharCounter) {
        counter++; // Increment counter on read
        pCharCounter->setValue(counter);
        Serial.printf("Counter read: %u\n", counter);

        // Toggle GPIO 2 based on counter parity
        digitalWrite(debugPin, (counter % 2) ? HIGH : LOW);
    }
};

void sendTimestamp(void* arg) {
	if (deviceConnected) {
		uint64_t now = esp_timer_get_time();
		pCharTimestamp->setValue((uint8_t *)&now, 8);
    //String message = "{\"now\":" + String(now) + "}\n";
    //pCharTimestamp->setValue(message.c_str());
		pCharTimestamp->notify();
		Serial.printf("Timestamp sent: %llu µs\n", now);
	}
}

void addCharacteristicDescription(BLECharacteristic *pChar, const char* description) {
    BLE2901 *pDesc = new BLE2901();
    pDesc->setDescription(String(description));
    pChar->addDescriptor(pDesc);
}


void setup() {
    Serial.begin(115200);
    pinMode(debugPin, OUTPUT);
    pinMode(togglePin, OUTPUT);
    digitalWrite(debugPin, LOW);
    digitalWrite(togglePin, LOW);
    uint32_t zero = 0;

    // Initialize BLE
    BLEDevice::init("ESP32_BLE_Server_B");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Timestamp Characteristic (notify every 60s)
    pCharTimestamp = pService->createCharacteristic(
        CHAR_TIMESTAMP_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharTimestamp->addDescriptor(new BLE2902());
    addCharacteristicDescription(pCharTimestamp, "Timestamp (µs since boot)");  // NEW

    // Counter Characteristic (read-triggered)
    pCharCounter = pService->createCharacteristic(
        CHAR_COUNTER_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    pCharCounter->setCallbacks(new CounterCharCallbacks());
    pCharCounter->setValue(zero);
    addCharacteristicDescription(pCharCounter, "Counter (increments on read)");  // NEW

    // Create a periodic timer (60s in microseconds)
    esp_timer_create_args_t timer_args = {
        .callback = &sendTimestamp,
        .name = "timestamp_timer"
    };
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 60 * 1000000); // 60s in µs

    // Start Service & Advertising
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Helps with iPhone connections 0x12 ili 0x0
    BLEDevice::startAdvertising();
    Serial.println("BLE Server Running!");
}

void loop() {
    static uint32_t lastToggleTime = 0;
    uint32_t currentTime = millis();

    // Toggle GPIO 22 every 2 seconds
    if (currentTime - lastToggleTime >= 2000) {
        lastToggleTime = currentTime;
        digitalWrite(togglePin, !digitalRead(togglePin)); // Toggle state
        Serial.printf("GPIO 22 toggled: %s\n", digitalRead(togglePin) ? "HIGH" : "LOW");
    }
    delay(500); // Prevent watchdog reset
}