#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "data_samples.h"  // Your predefined data

// BLE UUIDs
#define SERVICE_UUID        "4FAFC201-1FB5-459E-8FCC-C5C9C331914B"
#define DATA_CHAR_UUID      "A9B1C2D3-E4F5-1234-5678-9ABC12345678"
#define CONTROL_CHAR_UUID   "B2C3D4E5-F6A7-2345-6789-0BCD23456789"

// Buffer setup
#define LED_PIN 23  // GPIO 23
uint16_t dataBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
uint8_t dataIndex = 0;  // Tracks position in predefinedData

BLEServer *pServer;
BLECharacteristic *pDataCharacteristic;
BLECharacteristic *pControlCharacteristic;

bool deviceConnected = false;
bool ackReceived = false;

// BLE Callbacks (unchanged)
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        bufferIndex = 0;  // Reset on new connection
        dataIndex = 0;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

class ControlCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        if (pCharacteristic->getData()[0] == 1) {
            ackReceived = true;
        }
    }
};

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    
    // Initialize BLE
    BLEDevice::init("ESP32_Slave");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pDataCharacteristic = pService->createCharacteristic(
        DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pControlCharacteristic = pService->createCharacteristic(
        CONTROL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pControlCharacteristic->setCallbacks(new ControlCallback());
    
    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Slave Ready - Sampling at 10Hz");
}

void loop() {
    static unsigned long lastSampleTime = 0;
    unsigned long currentTime = millis();

    // Take one sample every 100ms (10Hz)
    if (currentTime - lastSampleTime >= 100) {
        lastSampleTime = currentTime;

        // Store current sample
        dataBuffer[bufferIndex] = predefinedData[dataIndex];
        bufferIndex++;
        dataIndex = (dataIndex + 1) % 100;  // Circular buffer for predefined data

        Serial.print("Sampled value: ");
        Serial.println(dataBuffer[bufferIndex-1]);

        if (dataBuffer[bufferIndex-1] >= 100) {
            digitalWrite(LED_PIN, HIGH);
        }
        else {
          digitalWrite(LED_PIN, LOW);
        }

        // Buffer full? Send it!
        if (bufferIndex >= BUFFER_SIZE) {
            if (deviceConnected) {
                pDataCharacteristic->setValue((uint8_t*)dataBuffer, sizeof(dataBuffer));
                pDataCharacteristic->notify();
                Serial.println("Full buffer sent!");

                // Wait for ACK before resetting
                while (!ackReceived) {
                    delay(10);
                }
                ackReceived = false;
                bufferIndex = 0;  // Reset buffer
            }
        }
    }
}