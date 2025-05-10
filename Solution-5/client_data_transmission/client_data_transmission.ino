#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include "esp_timer.h"
#include "esp_bt.h"
//#include "esp_bt_main.h"
//#include "esp_gap_ble_api.h"

// Server details (replace with your actual MAC)
#define ESP_A_MAC "EC:94:CB:4C:B8:76"
#define ESP_B_MAC "EC:94:CB:4A:7C:26"
#define ESP_C_MAC "88:13:BF:00:7E:06"
#define ESP_D_MAC "C0:5D:89:B0:75:BA"
static BLEAddress serverAddress(ESP_A_MAC);

// UUIDs (must match server)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914a"
#define TX_DATA_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (buffer data)
#define RX_CONTROL_UUID     "2c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Write (control)
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (timestamp)
#define LATENCY_UUID        "4c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify Write (latency)
#define ALERT_MEAS_UUID     "ac95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (incorrect data)

// BLE Client objects
BLEClient* pClient;
BLERemoteCharacteristic *pTxData;
BLERemoteCharacteristic *pRxControl;
BLERemoteCharacteristic *pTimestampChar;
BLERemoteCharacteristic *pLatencyControl; // For latency characteristic
BLERemoteCharacteristic *pAlertMeasurement; // For incorrect measurement

bool connected = false;
bool latencyTestComplete = false;
uint64_t avgLatency = 0;
bool startLatencyTest = false;
bool responseReceived = false;

int64_t previousDelta = 0;
int64_t timeDiff;
int64_t maxDelta = -__INT64_MAX__;

// UINT64 can store up to 300 000 years of up time, INT64 up to 150 000 years so there is no problem of overflowing :) 

// Alert times 
uint64_t alertTimestamp;



// Time tracking
uint64_t lastServerTimestamp = 0;
uint64_t connectionStartTime = 0;
int connectedPin = 22;


class ClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("Connected to server");
        connected = true;
        connectionStartTime = esp_timer_get_time();
        uint16_t mtu = pClient->getMTU();
        Serial.printf("Connected with MTU: %d\n", mtu);
    }

    void onDisconnect(BLEClient* pclient) {
        Serial.println("Disconnected from server");
        connected = false;
        latencyTestComplete = false;
        avgLatency = 0;
        startLatencyTest = false;
        responseReceived = false;
        BLEDevice::getScan()->clearResults();
        delay(100);

        // digitalWrite(connectedPin, LOW);
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

//
void timestampNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                           uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(uint64_t)) {
        lastServerTimestamp = *((uint64_t*)pData);
        uint64_t clientTime = esp_timer_get_time();
        
        int64_t timeDiff = clientTime - lastServerTimestamp; // result can be negative as well! 
        
        if (maxDelta != -__INT64_MAX__)
        {
            maxDelta = (timeDiff > maxDelta) ? timeDiff : maxDelta;
            
            Serial.printf("[Time Sync] Server: %llu μs | Client: %llu μs | Δ: %lld μs\n", 
                        lastServerTimestamp, clientTime, timeDiff);
            Serial.printf("Previous Δ - current Δ: %lld | max Δ: %lld μs\n", 
                        previousDelta - timeDiff, maxDelta);
            previousDelta = timeDiff;
        }
        else
        {
            maxDelta = (timeDiff > maxDelta) ? timeDiff : maxDelta;
            previousDelta = timeDiff;
            Serial.printf("[Time Sync] Server: %llu μs | Client: %llu μs | Δ: %lld μs\n", 
                        lastServerTimestamp, clientTime, timeDiff);
        }
    }
}

// Zaprimi alert time stamp. izracunaj server clock i client clock na osnovu diff-a.
// timeDiff = clientTime - serverTime
// 
void alertNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
    uint8_t* pData, size_t length, bool isNotify) 
{
    if (length == sizeof(uint64_t)) {
        alertTimestamp = *((uint64_t*)pData);

        uint64_t clientTime = esp_timer_get_time();
        int64_t calcClientTime = timeDiff + alertTimestamp;
        int64_t calcServerTime = calcClientTime - timeDiff;
        Serial.printf("[Alert Sync] Server alert time: %llu μs | Server calc alert time: %lld μs | Δ: %lld μs\n",
            alertTimestamp, calcServerTime, alertTimestamp - calcServerTime);
        Serial.printf("[Alert Sync] Client current time: %llu μs | Client calc alert time: %lld μs | Δ: %lld μs\n",
            clientTime, calcClientTime, clientTime - calcClientTime);
        digitalWrite(connectedPin, HIGH);
    }
}


bool connectToServer() {
    Serial.print("Connecting to ");
    Serial.println(serverAddress.toString().c_str());
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    // Set up MTU callback
    //pClient->setMTU(247);
  
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
    Serial.println("Service discovered successfully"); // Add this

    // std::map<std::string, BLERemoteService*>* services = pClient->getServices();
    // if(!services) {
    //     Serial.println("No services found");
    //     return false;
    // }

    // Serial.printf("Found %d services:\n", services->size());
    // for(auto it = services->begin(); it != services->end(); ++it) {
    //     Serial.printf("- Service UUID: %s\n", it->first.c_str());
    // } // TODO ovo ga rusi
    
    // Get the timestamp characteristic
    pTimestampChar = pRemoteService->getCharacteristic(TIMESTAMP_UUID);
    if (pTimestampChar == nullptr) {
        Serial.println("Failed to find timestamp characteristic");
        pClient->disconnect();
        return false;
    }


    delay(100);
    // Get the timestamp characteristic
    pLatencyControl = pRemoteService->getCharacteristic(LATENCY_UUID);
    if (pLatencyControl == nullptr) {
        Serial.println("Failed to find latency characteristic");
        pClient->disconnect();
        return false;
    }

    delay(100);
    // Get the alert characteristic
    pAlertMeasurement = pRemoteService->getCharacteristic(ALERT_MEAS_UUID);
    if (pAlertMeasurement == nullptr) {
        Serial.println("Failed to find alert characteristic");
        Serial.println("Available characteristics:");
        std::map<std::string, BLERemoteCharacteristic*>* pChars = pRemoteService->getCharacteristics();
        for(auto it = pChars->begin(); it != pChars->end(); it++) {
            Serial.println(it->first.c_str());
        }
        pClient->disconnect();
        return false;
        //Serial.println("Failed to find alert characteristic");
        //pClient->disconnect();
        //return false;
    }
    
    // Subscribe to notifications
    if(pTimestampChar->canNotify()) {
        pTimestampChar->registerForNotify(timestampNotifyCallback);
    }

    // Subscribe to notifications of the alert
    if(pAlertMeasurement->canNotify()) {
        pAlertMeasurement->registerForNotify(alertNotifyCallback);
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
    Serial.println("ESP32 BLE alert Client");

    // Initialize BLE
    BLEDevice::init("ESP32_Client_Data_Transmission"); // Using client MAC in name
    pinMode(connectedPin, OUTPUT);
    digitalWrite(connectedPin, LOW);

    String bleMac = BLEDevice::getAddress().toString().c_str();
    Serial.println("BLE MAC: " + bleMac);

    
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