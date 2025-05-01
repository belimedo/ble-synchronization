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
#define LATENCY_UUID        "4c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify Write (latency)

// BLE Client objects
BLEClient* pClient;
BLERemoteCharacteristic* pTimestampChar;
BLERemoteCharacteristic* pLatencyControl;
bool connected = false;
bool latencyTestComplete = false;
uint64_t avgLatency = 0;
bool startLatencyTest = false;
bool responseReceived = false;

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
        latencyTestComplete = false;
        avgLatency = 0;
        startLatencyTest = false;
        responseReceived = false;
        digitalWrite(connectedPin, LOW);
        // Clean up resources
        BLEDevice::deinit(true); // Full BLE stack reset
        delay(500);
        BLEDevice::init("ESP32_Timesinc_client"); // Reinitialize
        Serial.println("Reinitializing done.");
    }
};

void latencyNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(uint64_t)) {
        responseReceived = true;
        uint64_t serverEchoTime = *((uint64_t*)pData);
        if (serverEchoTime == 0)
        {
            startLatencyTest = true;
            Serial.println("Received 0s to PING. Starting the latency test.");
        }
        else
        {
            uint64_t roundtrip = esp_timer_get_time() - serverEchoTime;
            avgLatency += roundtrip / 2; // Accumulate one-way latency
            Serial.printf("Received response. Latency equals: %llu.\n", roundtrip / 2);
        }

    }
}


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

    // Get the timestamp characteristic
    pLatencyControl = pRemoteService->getCharacteristic(LATENCY_UUID);
    if (pLatencyControl == nullptr) {
        Serial.println("Failed to find latency characteristic");
        pClient->disconnect();
        return false;
    }
    
    // Subscribe to notifications
    if(pTimestampChar->canNotify()) {
        pTimestampChar->registerForNotify(timestampNotifyCallback);
    }

    // Subscribe to notifications
    if(pLatencyControl->canNotify()) {
        pLatencyControl->registerForNotify(latencyNotifyCallback);
    }
    
    Serial.println("Connected and subscribed to timestamp updates and latency tests.");
    return true;
}

void performLatencyTest() {
    const uint8_t testCount = 10;
    avgLatency = 0;
    pLatencyControl->writeValue("PING");
    Serial.println("Starting latency calibration...");

    while(!startLatencyTest) {delay(50);} // busy wait
    delay(500); // Space out tests
    for (int i = 0; i < testCount; i++) {
        uint64_t sendTime = esp_timer_get_time();
        pLatencyControl->writeValue((uint8_t*)&sendTime, sizeof(sendTime));
        
        Serial.printf("Send %d timestamp.", i + 1);
        delay(500); // Space out tests. Ovdje treba nekakav notify mehanizam interni a ne ovako
        Serial.println("Waiting for response.");
        while(!responseReceived) {delay(10);}
        responseReceived = false;
    }
    
    avgLatency /= testCount;
    Serial.printf("Calibration complete. Avg latency: %llu μs\n", avgLatency);
    
    // Switch to timestamp mode
    pLatencyControl->writeValue("START_TIMESTAMPS");
    latencyTestComplete = true;
    startLatencyTest = false;
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

    performLatencyTest();
}

void loop() {
    if (!connected) {
        Serial.println("Attempting reconnection...");
        if (connectToServer()) {
            Serial.println("Reconnected successfully");
            performLatencyTest();
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