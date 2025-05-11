#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include "esp_timer.h"
#include "esp_bt.h"
#include "../constants.h"
// #include "../communication_structures.h"
//#include "esp_gap_ble_api.h"

#define NUMBER_OF_STORE_BUFFERS (3) // Three buffers to store 30 seconds of data.
#define NUMBER_OF_SEND_BUFFERS  (2) // One for future samples one for past samples.
#define SAMPLING_FREQUENCY      (5) // Frekvencija samplovanja 
#define TIME_STORED             (30) // 30 sekundi se cuva u jednom store bufferu
#define TIME_SENT               (5)
#define STORE_BUFFER_SIZE       (TIME_STORED * SAMPLING_FREQUENCY * sizeof(uint8_t))
#define SEND_BUFFER_SIZE        ((TIME_SENT * SAMPLING_FREQUENCY + 1) * sizeof(uint8_t)) // Add one extra time slice, to be on the safe side

typedef struct storeBuffer 
{
    uint16_t    sampleIdx;
    uint64_t    startTimeStamp;
    uint8_t     buffer[STORE_BUFFER_SIZE];
} STORE_BUFFER;

typedef struct dataRequest
{
    //! 1 - Client sends out alert time; 2 - Client parsed previous data transfer; 3 - End of transfer;
    uint8_t     command; 
    // How much time we need to transfer.
    uint8_t     bufferSizeInSeconds; 
    // Fallback mechanism for MTU sizes.
    uint16_t    chunkSize;
    // Time on server at which alert happened calculated by the client (scaling parameter, for multiple servers). 
    uint64_t    alertTimeStamp;
} DATA_REQUEST;

static BLEAddress serverAddress(ESP_A_MAC);

// Debug handling stuff
bool DEBUG_PRINTS = true;

// BLE Client objects
BLEClient* pClient;
BLERemoteCharacteristic *pTxData;
BLERemoteCharacteristic *pRxControl;
BLERemoteCharacteristic *pTimestampChar;
// BLERemoteCharacteristic *pLatencyControl; // For latency characteristic
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

// Message reconstruction chunk size
uint16_t CHUNK_SIZE = DEFAULT_CHUNK_SIZE; // Default chunk size is 20. Assign new value according to the MTU size (MTU - 3)

// Data transfer
// uint8_t receivedBuffer[SEND_BUFFER_SIZE]; TODO: Ovo je za iduci
uint8_t receivedBuffer[STORE_BUFFER_SIZE];
// uint8_t printBuffer[SEND_BUFFER_SIZE];
uint8_t printBuffer[STORE_BUFFER_SIZE];
uint16_t receivedBytes = 0;
bool transferActive = false;

// Time tracking
uint64_t lastServerTimestamp = 0;
uint64_t connectionStartTime = 0;
int connectedPin = 22;


class ClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("Connected to server");
        connected = true;
        connectionStartTime = esp_timer_get_time();
        pClient->setMTU(517);
        uint16_t mtu = pClient->getMTU();
        Serial.printf("Connected with MTU: %d\n", mtu);
        CHUNK_SIZE = mtu - 3;
        Serial.printf("[Client] chunk size: %d\n", CHUNK_SIZE);
    }

    void onDisconnect(BLEClient* pclient) {
        Serial.println("Disconnected from server");
        connected = false;
        latencyTestComplete = false;
        avgLatency = 0;
        CHUNK_SIZE = DEFAULT_CHUNK_SIZE;
        startLatencyTest = false;
        responseReceived = false;
        BLEDevice::getScan()->clearResults();
        delay(100);

        // digitalWrite(connectedPin, LOW);
        // Clean up resources
        BLEDevice::deinit(true); // Full BLE stack reset
        delay(500);

        BLEDevice::init("ESP32_Client_Alert_Data_Transmission"); // Reinitialize
        Serial.println("Reinitializing done.");
    }
};

void printHexValues()
{
    Serial.println("Printing received hex values:");
    // for (int i = 0; i < SEND_BUFFER_SIZE; i+=SAMPLING_FREQUENCY)
    for (int i = 0; i < STORE_BUFFER_SIZE; i+=SAMPLING_FREQUENCY)
    {
        Serial.printf("%d: 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n", 
            i, printBuffer[i], printBuffer[i+1], printBuffer[i+2], printBuffer[i+3], printBuffer[i+4]); 
    }
};

//
void dataTransferNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                           uint8_t* pData, size_t length, bool isNotify) 
{
        if(!transferActive) return;

        uint8_t* data = (uint8_t *)pData;
        
        memcpy(&receivedBuffer[receivedBytes], data, length);
        receivedBytes += length;

        Serial.printf("Received %d bytes (Total: %d/%d)\n", 
                     length, receivedBytes, STORE_BUFFER_SIZE);
                    //  length, receivedBytes, SEND_BUFFER_SIZE);

        // Request next chunk if necessary.
        // if (receivedBytes < SEND_BUFFER_SIZE)
        if (receivedBytes < STORE_BUFFER_SIZE)
        {
            Serial.println("Requesting new chunk!");
            DATA_REQUEST request = {2, 0, CHUNK_SIZE, 0};
            pRxControl->writeValue((uint8_t*)&request, sizeof(request), true);
        }
        // else if(receivedBytes >= SEND_BUFFER_SIZE) 
        else if(receivedBytes >= STORE_BUFFER_SIZE) 
        {
            transferActive = false;
            Serial.println("Transfer complete!");
            
            // Send transfer complete command
            DATA_REQUEST endRequest = {3, 0, 0, 0};
            pRxControl->writeValue((uint8_t*)&endRequest, sizeof(endRequest), true);
            // memcpy(printBuffer, receivedBuffer, SEND_BUFFER_SIZE);
            memcpy(printBuffer, receivedBuffer, STORE_BUFFER_SIZE);
            printHexValues();
        }
}


// TODO: Preci na klasni callback
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

bool readyForTransfer = false;

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
        Serial.println("Requesting data transfer!");
        Serial.println("Request sent!");
        Serial.printf("Time sent: %llu\n", alertTimestamp);
        readyForTransfer = true;
        // transferActive = true;
        // receivedBytes = 0;
        // DATA_REQUEST request = {1, STORE_BUFFER_SIZE, CHUNK_SIZE, alertTimestamp}; 
        // Serial.printf("Sent request: Command %d, Alert TS: %llu\n, chunk size: %d", 
        //         request.command, request.alertTimeStamp, request.chunkSize);
        // pRxControl->writeValue((uint8_t*)&request, sizeof(DATA_REQUEST), true);

        // requestDataTransfer(alertTimestamp); // TODO: Potencijalni problem nakon 130 godina.
    }
}



void requestDataTransfer(uint64_t alertTime) 
{
    // DATA_REQUEST request = {1, SEND_BUFFER_SIZE, CHUNK_SIZE, alertTime}; // TODO: Traziti samo malo, ne citavih 30 sekundi ali za ovaj solution je dovoljno samo poslati dati bafer! 
    DATA_REQUEST request = {1, STORE_BUFFER_SIZE, CHUNK_SIZE, alertTime}; 
    pRxControl->writeValue((uint8_t*)&request, sizeof(request), true);
    Serial.println("Request sent!");
    Serial.printf("Time sent: %llu\n", alertTime);
    transferActive = true;
    receivedBytes = 0;
    readyForTransfer = false;
}

void printAllCharacteristics(BLEClient* pClient) {
    // Get all services from the server
    std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
    
    if (pServices->empty()) {
      Serial.println("No services found!");
      return;
    }
  
    Serial.println("\n===== Available Characteristics =====");
    
    // Iterate through each service
    for (auto serviceIterator = pServices->begin(); serviceIterator != pServices->end(); ++serviceIterator) {
      BLERemoteService* pRemoteService = serviceIterator->second;
      Serial.printf("\nService UUID: %s\n", pRemoteService->getUUID().toString().c_str());
  
      // Get all characteristics for this service
      std::map<std::string, BLERemoteCharacteristic*>* pChars = pRemoteService->getCharacteristics();
      
      if (pChars->empty()) {
        Serial.println("  No characteristics in this service");
        continue;
      }
  
      // Print each characteristic
      for (auto charIterator = pChars->begin(); charIterator != pChars->end(); ++charIterator) {
        BLERemoteCharacteristic* pChar = charIterator->second;
        
        Serial.printf("  Characteristic UUID: %s\n", pChar->getUUID().toString().c_str());
        Serial.printf("    Handle: %d\n", pChar->getHandle());
        Serial.printf("    Properties: %s%s%s%s%s\n",
          (pChar->canRead() ? "READ " : ""),
          (pChar->canWrite() ? "WRITE " : ""),
          (pChar->canNotify() ? "NOTIFY " : ""),
          (pChar->canIndicate() ? "INDICATE " : ""),
          (pChar->canBroadcast() ? "BROADCAST" : "")
        );
      }
    }
    Serial.println("===================================");
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

    // Set up MTU callback
    pClient->setMTU(512);
    
    // Get the service
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("Failed to find service");
        pClient->disconnect();
        return false;
    }
    Serial.println("Service discovered successfully"); // Add this

    // pClient->getServices();
    // printAllCharacteristics(pClient);


    // std::map<std::string, BLERemoteService*>* services = pClient->getServices();
    // if(!services) {
    //     Serial.println("No services found");
    //     return false;
    // }

    // Get the timestamp characteristic
    pTimestampChar = pRemoteService->getCharacteristic(TIMESTAMP_UUID);
    if (pTimestampChar == nullptr) {
        Serial.println("Failed to find timestamp characteristic");
        pClient->disconnect();
        return false;
    }


    delay(100);
    // Get the timestamp characteristic
    // pLatencyControl = pRemoteService->getCharacteristic(LATENCY_UUID);
    // if (pLatencyControl == nullptr) {
    //     Serial.println("Failed to find latency characteristic");
    //     pClient->disconnect();
    //     return false;
    // }

    // delay(100);
    // Get TX data characteristic (This will read data)
    pTxData = pRemoteService->getCharacteristic(TX_DATA_UUID);
    if (pTxData == nullptr) {
        Serial.println("Failed to find transfer data characteristic");
        pClient->disconnect();
        return false;
    }

    delay(100);
    // Get RX Controll characteristic. (Sending out commands to a server)
    pRxControl = pRemoteService->getCharacteristic(RX_CONTROL_UUID);
    if (pRxControl == nullptr) {
        Serial.println("Failed to find control characteristic");
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
        Serial.println("Registered for timestamp notify.");
        pTimestampChar->registerForNotify(timestampNotifyCallback);
    }

    // Subscribe to notifications of the alert
    if(pAlertMeasurement->canNotify()) {
        Serial.println("Registered for alert notify.");
        pAlertMeasurement->registerForNotify(alertNotifyCallback);
    }

    if(pRxControl->canWrite() && pRxControl->canWriteNoResponse())
    {
        Serial.println("Registered for write on control.");
    }

    // Subscribe to notifications
    // if(pLatencyControl->canNotify()) {
    //     pLatencyControl->registerForNotify(latencyNotifyCallback);
    // }
    
    // Subscribe to notifications for data transfer
    if(pTxData->canNotify()) {
        Serial.println("Registered for data transfer notify.");
        pTxData->registerForNotify(dataTransferNotifyCallback);
    }
    
    Serial.println("Connected and subscribed to timestamp updates and latency tests.");
    return true;
}

// void performLatencyTest() {
//     const uint8_t testCount = 10;
//     avgLatency = 0;
//     pLatencyControl->writeValue("PING");
//     Serial.println("Starting latency calibration...");

//     while(!startLatencyTest) {delay(50);} // busy wait
//     delay(500); // Space out tests
//     for (int i = 0; i < testCount; i++) {
//         uint64_t sendTime = esp_timer_get_time();
//         pLatencyControl->writeValue((uint8_t*)&sendTime, sizeof(sendTime));
        
//         Serial.printf("Send %d timestamp.", i + 1);
//         delay(500); // Space out tests. Ovdje treba nekakav notify mehanizam interni a ne ovako
//         Serial.println("Waiting for response.");
//         while(!responseReceived) {delay(10);}
//         responseReceived = false;
//     }
    
//     avgLatency /= testCount;
//     Serial.printf("Calibration complete. Avg latency: %llu μs\n", avgLatency);
    
//     // Switch to timestamp mode
//     // pLatencyControl->writeValue("START_TIMESTAMPS");
//     latencyTestComplete = true;
//     startLatencyTest = false;
// }


void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE alert Client");

    // Initialize BLE
    BLEDevice::init("ESP32_Client_Alert_Data_Transmission"); // Using client MAC in name
    pinMode(connectedPin, OUTPUT);
    digitalWrite(connectedPin, LOW);

    // memset(receivedBuffer, 0, SEND_BUFFER_SIZE);
    // memset(printBuffer, 0, SEND_BUFFER_SIZE);
    memset(receivedBuffer, 0, STORE_BUFFER_SIZE);
    memset(printBuffer, 0, STORE_BUFFER_SIZE);

    String bleMac = BLEDevice::getAddress().toString().c_str();
    Serial.println("BLE MAC: " + bleMac);

    
    // Attempt connection
    while (!connectToServer()) {
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
    }

    // performLatencyTest();
}

void loop() {
    if (!connected) {
        Serial.println("Attempting reconnection...");
        if (connectToServer()) {
            Serial.println("Reconnected successfully");
            // performLatencyTest();
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
    
    if (readyForTransfer)
    {
        Serial.println("Entering the transfer loop!");
        requestDataTransfer(alertTimestamp);
    }

    delay(20); // Prevent watchdog triggers
}