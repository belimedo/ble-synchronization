#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include "esp_timer.h"
#include "esp_bt.h"
#include "../constants.h"
#include "../communication_structures.h"
//#include "esp_gap_ble_api.h"

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
bool disconnected = true;
bool latencyTestComplete = false;
uint64_t avgLatency = 0;
bool startLatencyTest = false;
bool responseReceived = false;
bool readyForTransfer = false;

DATA_REQUEST request = {0};

int64_t previousDelta = 0;
int64_t timeDiff;
int64_t maxDelta = -__INT64_MAX__;

// UINT64 can store up to 300 000 years of up time, INT64 up to 150 000 years so there is no problem of overflowing :) 

// Alert times 
uint64_t alertTime;
uint64_t serverAlertTimeStamp;
int64_t clientAlertTime;

// Message reconstruction chunk size
// uint16_t CHUNK_SIZE = DEFAULT_CHUNK_SIZE; // Default chunk size is 20. Assign new value according to the MTU size (MTU - 3)

// Data transfer
uint8_t receivedBuffer[sizeof(sendBuffer)]; // Primamo uint8 podatak koji kasnije moramo kastovati
uint8_t printBuffer[SEND_BUFFER_SIZE]; // ovo je samo bagfer 
sendBuffer readData;
uint16_t receivedBytes = 0;
bool transferActive = false;

// Time tracking
uint64_t lastServerTimestamp = 0;
uint64_t connectionStartTime = 0;
int connectedPin = 22;


void setMtuSize(BLEClient* pclient)
{
    pClient->setMTU(START_MTU_SIZE);
    uint16_t mtu = pClient->getMTU();
    Serial.printf("Connected with MTU: %d\n", mtu);
    // CHUNK_SIZE = mtu - 3;
    Serial.printf("[Client] Max possible chunk size: %d\n", mtu - 3);
}


class ClientCallbacks : public BLEClientCallbacks 
{
    void onConnect(BLEClient* pclient) 
    {
        Serial.println("Connected to server");
        connected = true;
        disconnected = false;
        connectionStartTime = esp_timer_get_time();
        Serial.println("Setting up MTU onConnect.");
        setMtuSize(pclient);
    }

    void onDisconnect(BLEClient* pclient) {
        Serial.println("Disconnected from server");
        connected = false;
        latencyTestComplete = false;
        avgLatency = 0;
        // CHUNK_SIZE = DEFAULT_CHUNK_SIZE;
        startLatencyTest = false;
        responseReceived = false;
        // BLEDevice::getScan()->clearResults();
        // delay(100);

        // digitalWrite(connectedPin, LOW);
        // Clean up resources
        // Serial.println("Deinit in progress");
        // BLEDevice::deinit(true); // Full BLE stack reset
        // delay(400);

        // Serial.println("Init in progress");
        // BLEDevice::init("ESP32_Client_Alert_Data_Transmission"); // Reinitialize
        // Serial.println("Reinitializing done.");
        disconnected = true;
    }
};

void printHexValues()
{
    Serial.println("Printing received hex values:");
    Serial.printf("Start time of the data: %llu\n Data samples:",readData.timeStamp);
    for (int i = 0; i < SEND_BUFFER_SIZE; i++)
    {
        if ((i % SAMPLING_FREQUENCY) == 0)
        {
             Serial.printf("\n");
        }
        Serial.printf("%d: 0x%2x, ", i, readData.dataBufferPtr[i]);
    }
    Serial.printf("\n");
}

//
void dataTransferNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                           uint8_t* pData, size_t length, bool isNotify) 
{
    // If there is no communication started prior to this call, return back
    if(!transferActive) return;

    // Copy  data directly to receivedBuffer
    // TODO: Add semaphore maybe?
    memcpy(&receivedBuffer[receivedBytes], pData, length);
    receivedBytes += length;

    Serial.printf("Received %d bytes (Total: %d/%d)\n", 
                    length, receivedBytes, sizeof(sendBuffer));

    // Request next chunk if necessary.
    if (receivedBytes < sizeof(sendBuffer))
    {
        Serial.println("Requesting new chunk!");
        request.command = 2;
    }
    else if(receivedBytes >= sizeof(sendBuffer))
    {
        Serial.println("Transfer complete! Casting to sendBuffer struct.");
        // Send transfer complete command
        request.command = 3;
        memcpy(&readData, receivedBuffer, sizeof(readData));
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
void alertNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
    uint8_t* pData, size_t length, bool isNotify) 
{
    if (length == sizeof(uint64_t)) {
        alertTime = *((uint64_t*)pData);

        uint64_t clientTime = esp_timer_get_time();
        clientAlertTime = timeDiff + alertTime;
        serverAlertTimeStamp = clientAlertTime - timeDiff;
        Serial.printf("[Alert Sync] Server alert time: %llu μs | Server calc alert time: %lld μs | Δ: %lld μs\n",
            alertTime, serverAlertTimeStamp, alertTime - serverAlertTimeStamp);
        Serial.printf("[Alert Sync] Client current time: %llu μs | Client calc alert time: %lld μs | Δ: %lld μs\n",
            clientTime, clientAlertTime, clientTime - clientAlertTime);
        digitalWrite(connectedPin, HIGH);
        Serial.println("Requesting data transfer!");
        Serial.println("Request sent!");
        Serial.printf("Time sent: %llu\n", serverAlertTimeStamp);
        readyForTransfer = true;
        memset(&readData, 0, sizeof(sendBuffer));
    }
}


void sendTransferRequest() 
{
    pRxControl->writeValue((uint8_t*)&request, sizeof(request), true);
    Serial.println("Request sent!");
    Serial.printf("Request command: %d, alert time sent: %llu\n", request.command, request.alertTimeStamp);
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
    if (!pClient->connect(serverAddress, BLE_ADDR_TYPE_PUBLIC, 2000)) {
        Serial.println("Connection failed");
        return false;
    }

    // Set up MTU callback
    // setMtuSize(pClient);
    // pClient->setMTU(512);
    
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
    memset(receivedBuffer, 0, sizeof(sendBuffer));
    memset(printBuffer, 0, SEND_BUFFER_SIZE);
    memset(&request, 0, sizeof(DATA_REQUEST));

    String bleMac = BLEDevice::getAddress().toString().c_str();
    Serial.println("BLE MAC: " + bleMac);

    
    // Attempt connection
    while (!connectToServer()) {
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
    }

    setMtuSize(pClient);

    // performLatencyTest();
}

void loop() {
    if (!connected && disconnected) {
        Serial.println("Attempting reconnection...");
        if (connectToServer()) 
        {
            Serial.println("Reconnected successfully");
            // performLatencyTest();
        }
        else
        {
            delay(2000);
        }
        
    }
    
    // Print connection duration periodically
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 10000) { // Every 10 seconds
        if (connected && !disconnected) {
            uint64_t uptime = (esp_timer_get_time() - connectionStartTime) / 1000000;
            Serial.printf("Connection uptime: %llu seconds\n", uptime);
        }
        lastPrint = millis();
    }
    
    if (transferActive || readyForTransfer)
    {
        Serial.println("Entering the transfer loop!");
        if (readyForTransfer) // the first transfer of a series
        {
            readyForTransfer = false;
            transferActive = true;
            receivedBytes = 0;
            request.command = 1;
            request.bufferSizeInSeconds = TIME_SENT;
            request.chunkSize =  pClient->getMTU() - 3;
            request.alertTimeStamp = serverAlertTimeStamp;
            sendTransferRequest();
        }
        else if (request.command == 2)
        {
            sendTransferRequest();
        }
        else if (request.command == 3)
        {
            sendTransferRequest();
            transferActive = false;
            printHexValues();
        }
    }

    delay(100); // Prevent watchdog triggers
}