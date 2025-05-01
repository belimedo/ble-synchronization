#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include "esp_timer.h"

// Server details (replace with your actual MAC)
#define SERVER_MAC "EC:94:CB:4A:7C:26"
static BLEAddress serverAddress(SERVER_MAC);

// UUIDs (must match server)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c"

// BLE Client objects
BLEClient* pClient;
BLERemoteCharacteristic* pTimestampChar;
bool connected = false;

// Time tracking
uint64_t lastServerTimestamp = 0;
uint64_t connectionStartTime = 0;
int connectedPin = 22;

class ClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("Connected to server");
        connected = true;
        digitalWrite(connectedPin, HIGH);
        connectionStartTime = esp_timer_get_time();
    }

    void onDisconnect(BLEClient* pclient) {
        Serial.println("Disconnected from server");
        connected = false;
        digitalWrite(connectedPin, LOW);
        // Clean up resources
        BLEDevice::deinit(true); // Full BLE stack reset
        delay(500);
        BLEDevice::init("ESP32_Timesinc_client"); // Reinitialize
    }
};

void timestampNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                           uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(uint64_t)) {
        lastServerTimestamp = *((uint64_t*)pData);
        uint64_t clientTime = esp_timer_get_time();
        
        int64_t timeDiff = clientTime - lastServerTimestamp; // result can be negative as well! 
        
        Serial.printf("[Time Sync] Server: %llu μs | Client: %llu μs | Δ: %lld μs\n", 
                     lastServerTimestamp, clientTime, timeDiff);
    }
}

bool connectToServer() {
    Serial.print("Connecting to ");
    Serial.println(serverAddress.toString().c_str());
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    // // Connection parameters (improves reliability) TODO: Ovo zajebava jako
    // pClient->setConnectionParams(
    //     12,  // min interval (1.25ms units) = 15ms
    //     24,  // max interval (1.25ms units) = 30ms
    //     0,   // latency
    //     400  // timeout (10ms units) = 4s
    // );
    
    // // Connect with timeout
    // if (!pClient->connect(BLEAddress(SERVER_MAC), BLE_ADDR_TYPE_RANDOM, 10000)) {
    //     Serial.println("Connection timed out");
    //     return false;
    // }
    
    // Connect to the server
    if (!pClient->connect(serverAddress)) {
        Serial.println("Connection failed");
        return false;
    }
    
    // Get the service
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("Failed to find service");
        pClient->disconnect();
        return false;
    }
    
    // Get the timestamp characteristic
    pTimestampChar = pRemoteService->getCharacteristic(TIMESTAMP_UUID);
    if (pTimestampChar == nullptr) {
        Serial.println("Failed to find timestamp characteristic");
        pClient->disconnect();
        return false;
    }
    
    // Subscribe to notifications
    if(pTimestampChar->canNotify()) {
        pTimestampChar->registerForNotify(timestampNotifyCallback);
    }
    
    Serial.println("Connected and subscribed to timestamp updates");
    return true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE Timestamp Client");
    
    // Initialize BLE
    BLEDevice::init("ESP32_Timesinc_client"); // Using client MAC in name
    pinMode(connectedPin, OUTPUT);
    digitalWrite(connectedPin, LOW);
    
    // Attempt connection
    while (!connectToServer()) {
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
    }
}

void loop() {
    if (!connected) {
        Serial.println("Attempting reconnection...");
        if (connectToServer()) {
            Serial.println("Reconnected successfully");
        }
        delay(5000);
    }
    
    // Print connection duration periodically
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 10000) { // Every 10 seconds
        if (connected) {
            uint64_t uptime = (esp_timer_get_time() - connectionStartTime) / 1000000;
            Serial.printf("Connection uptime: %llu seconds\n", uptime);
        }
        lastPrint = millis();
    }
    
    delay(100); // Prevent watchdog triggers
}