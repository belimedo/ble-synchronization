#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "esp_bt.h"
#include "../constants.h"
#include "../communication_structures.h"
//#include "esp_gap_ble_api.h"

static BLEAddress serverAddress(ESP_A_MAC);

#define RECONSTRUCTION_TIME_US          (60000000) // 60 seconds 60 * 1'000'000

// Debug handling stuff
bool DEBUG_PRINTS = true;

// BLE Client objects
BLEClient* pClient = nullptr;
BLERemoteCharacteristic *pTxData;
BLERemoteCharacteristic *pRxControl;
BLERemoteCharacteristic *pTimestampChar;
// BLERemoteCharacteristic *pLatencyControl; // For latency characteristic
BLERemoteCharacteristic *pAlertMeasurement; // For incorrect measurement

volatile bool connected = false;
volatile bool disconnected = true;
volatile bool latencyTestComplete = false;
uint64_t avgLatency = 0;
volatile bool timestampCompleted = false;
volatile bool responseReceived = false;
volatile bool readyForTransfer = false;
volatile bool reconstructionTriggered = false;

SemaphoreHandle_t receiveBufferMutex[NUMBER_OF_SEND_BUFFERS];

// timer for reconstruction
esp_timer_handle_t reconstructionTimer;

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
sendBuffer readData;
sendBuffer readDataBuffers[NUMBER_OF_SEND_BUFFERS];
uint8_t buffersReceived = 0;
uint16_t receivedBytes = 0;
volatile bool transferActive = false;
volatile bool sendCommand = false;

// Time tracking
uint64_t lastServerTimestamp = 0;


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
        reconstructionTriggered = false;
        Serial.println("[onConnect] Setting up MTU.");
        setMtuSize(pclient);
    }

    void onDisconnect(BLEClient* pclient) {
        Serial.println("Disconnected from server");
        connected = false;
        latencyTestComplete = false;
        avgLatency = 0;
        timestampCompleted = false;
        responseReceived = false;
        disconnected = true;
        reconstructionTriggered = false;
        esp_timer_stop(reconstructionTimer);
    }
};

void printPowerValues()
{
    for (int sendingBufferIdx = 0; sendingBufferIdx < NUMBER_OF_SEND_BUFFERS; sendingBufferIdx++)
    {
        Serial.printf("[Print power] Printing %s buffer.\n", sendingBufferIdx % 2 == 0 ? "past" : "future");
        if(xSemaphoreTake(receiveBufferMutex[sendingBufferIdx], 5) == pdTRUE)
        {
            Serial.printf("[Print power] Start time of the data: %llu\n Data samples:\n", readDataBuffers[sendingBufferIdx].timeStamp);
            for (int i = 0; i < SEND_BUFFER_ELEMENTS; i++)
            {
                Serial.printf("%d: 0x%4x [A] - 0x%4x [V] p = 0x%4x\n",
                    i, readDataBuffers[sendingBufferIdx].currentBuffer[i], readDataBuffers[sendingBufferIdx].voltageBuffer[i], 
                    abs(readDataBuffers[sendingBufferIdx].currentBuffer[i] - readDataBuffers[sendingBufferIdx].voltageBuffer[i]));
            }
            xSemaphoreGive(receiveBufferMutex[sendingBufferIdx]);
        }
        else
        {
            Serial.println("Couldn't access lock in print hex values.");
        }
    }
}


void powerReconstruction(void* arg)
{
    // Ukljuci se samo ukoliko je transfer inactive
    if (!transferActive)
    {
        Serial.println("[Reconstruct] Reconstruction callback");
        uint64_t clientTime = esp_timer_get_time();
        serverAlertTimeStamp = clientTime - timeDiff;
        reconstructionTriggered = true;
        readyForTransfer = true;
    }
}

// Callback to respond to the voltage and current data transfer
void dataTransferNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{
    // If there is no communication started prior to this call, return back
    if(!transferActive) return;

    memcpy(&receivedBuffer[receivedBytes], pData, length);
    receivedBytes += length;

    if (DEBUG_PRINTS)
    {
        Serial.printf("[Data transfer] Received %d bytes (Total: %d/%d)\n", length, receivedBytes, sizeof(sendBuffer));
    }
// receiveBufferMutex Add it here
    // Request next chunk if necessary.
    if (receivedBytes < sizeof(sendBuffer))
    {
        Serial.println("[Data transfer] Requesting new chunk!");
        request.command = 2;
        sendCommand = true;
    }
    else if((receivedBytes >= sizeof(sendBuffer)))
    {
        if(xSemaphoreTake(receiveBufferMutex[buffersReceived], 5) == pdTRUE)
        {
            if (buffersReceived == 0)
            {
                Serial.printf("[Data transfer] Past buffer transfer complete! Buffers received: %d/%d\n", buffersReceived + 1, NUMBER_OF_SEND_BUFFERS);
                // Send transfer complete command
                request.command = 3;
                memcpy(&readDataBuffers[buffersReceived], receivedBuffer, sizeof(sendBuffer)); // Ovdje mozda staviti receivedBytes umjesto sizeof(sendBuffer)?
                receivedBytes = 0;
            }
            else if (buffersReceived == 1) // moze i obicni else
            {
                Serial.printf("[Data transfer] Transfer complete! Buffers received: %d/%d\n", buffersReceived + 1, NUMBER_OF_SEND_BUFFERS);
                // Send transfer complete command
                request.command = 4;
                memcpy(&readDataBuffers[buffersReceived], receivedBuffer, sizeof(sendBuffer));
            }
            sendCommand = true;
            xSemaphoreGive(receiveBufferMutex[buffersReceived]);
            buffersReceived = (buffersReceived + 1) % NUMBER_OF_SEND_BUFFERS;
        }
        else
        {
            Serial.printf("[Data transfer] Unable to lock receive buffer %d.\n", buffersReceived);
        }
    }
}

//
void timestampNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{
    if (length == sizeof(uint64_t)) {
        lastServerTimestamp = *((uint64_t*)pData);
        uint64_t clientTime = esp_timer_get_time();
        
        timeDiff = clientTime - lastServerTimestamp; // result can be negative as well! 
        
        if (!timestampCompleted)
        {
            maxDelta = (timeDiff > maxDelta) ? timeDiff : maxDelta;
            if (DEBUG_PRINTS)
            {
                Serial.printf("[Time Sync] Server time: %llu μs | Client time: %llu μs | Δ: %lld μs\n", 
                            lastServerTimestamp, clientTime, timeDiff);
                Serial.printf("[Time Sync] Δ difference: %lld | max Δ: %lld μs\n", 
                            previousDelta - timeDiff, maxDelta);
                Serial.println("Starting reconstruction timer.");
            }
            previousDelta = timeDiff;
            timestampCompleted = true;
            esp_timer_start_periodic(reconstructionTimer, RECONSTRUCTION_TIME_US); // Periodic timer for sinusoid reconstruction.
        }
        else
        {
            maxDelta = (timeDiff > maxDelta) ? timeDiff : maxDelta;
            previousDelta = timeDiff;
            if (DEBUG_PRINTS)
            {
                Serial.printf("[Time Sync] Server time:: %llu μs | Client time:: %llu μs | Δ: %lld μs\n", 
                            lastServerTimestamp, clientTime, timeDiff);
                Serial.printf("[Time Sync] Δ difference: %lld | max Δ: %lld μs\n", 
                            previousDelta - timeDiff, maxDelta);
            }

        }
        // if (!transferActive)
        // {
        //     Serial.println("[Reconstruction] Sending reconstruction request.");
        //     serverAlertTimeStamp = clientTime - timeDiff;
        //     readyForTransfer = true;
        //     reconstructionTriggered = true;
        // }
    }
    
}

// Zaprimi alert time stamp. izracunaj server clock i client clock na osnovu diff-a.
// timeDiff = clientTime - serverTime
// 
void alertNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
    uint8_t* pData, size_t length, bool isNotify) 
{
    // Onemoguci alert ako je transfer vec u toku!
    if (length == sizeof(uint64_t) && !transferActive) {
        alertTime = *((uint64_t*)pData);

        uint64_t clientTime = esp_timer_get_time();
        clientAlertTime = timeDiff + alertTime;
        serverAlertTimeStamp = clientAlertTime - timeDiff;
        if (DEBUG_PRINTS)
        {
            Serial.printf("[Alert Sync] Server alert time: %llu μs | Server calc alert time: %lld μs | Δ: %lld μs\n",
                alertTime, serverAlertTimeStamp, alertTime - serverAlertTimeStamp);
            Serial.printf("[Alert Sync] Client current time: %llu μs | Client calc alert time: %lld μs | Δ: %lld μs\n",
                clientTime, clientAlertTime, clientTime - clientAlertTime);
            Serial.printf("[Alert Sync] Requesting data transfer for time: %llu\n", serverAlertTimeStamp);
        }
        readyForTransfer = true;
        memset(readDataBuffers, 0, sizeof(readDataBuffers));
    }
}


void sendTransferRequest() 
{
    pRxControl->writeValue((uint8_t*)&request, sizeof(request), true);
    if (DEBUG_PRINTS)
    {
        Serial.println("Request sent!");
        Serial.printf("Request command: %d, alert time sent: %llu\n", request.command, request.alertTimeStamp);
    }
}


void printAllCharacteristics(BLEClient* pClient) 
{
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


bool connectToServer() 
{
    Serial.print("Connecting to ");
    Serial.println(serverAddress.toString().c_str());
    if (pClient != nullptr) 
    {
        if (pClient->isConnected()) 
        {
            pClient->disconnect();
        }
        delete pClient;
        pClient = nullptr;
    }
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    // Connect to the server
    if (!pClient->connect(serverAddress, BLE_ADDR_TYPE_PUBLIC , 1000)) {
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

    // Get the timestamp characteristic
    pTimestampChar = pRemoteService->getCharacteristic(TIMESTAMP_UUID);
    if (pTimestampChar == nullptr) {
        Serial.println("Failed to find timestamp characteristic");
        pClient->disconnect();
        return false;
    }
    delay(100);


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

    // Subscribe to notifications for data transfer
    if(pTxData->canNotify()) {
        Serial.println("Registered for data transfer notify.");
        pTxData->registerForNotify(dataTransferNotifyCallback);
    }
    
    Serial.println("Connected and subscribed to timestamp updates and latency tests.");
    return true;
}


void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE alert Client");

    memset(receivedBuffer, 0, sizeof(sendBuffer));
    memset(readDataBuffers, 0, sizeof(readDataBuffers));
    memset(&request, 0, sizeof(DATA_REQUEST));

    for(int i = 0; i < NUMBER_OF_SEND_BUFFERS; i++) 
    {
        // kreiraj mutekse
        receiveBufferMutex[i] = xSemaphoreCreateMutex();
        if(receiveBufferMutex[i] == NULL) {
            Serial.println("Receive mutex creation failed!");
            while(1); // Critical failure
        }
    }
    if (DEBUG_PRINTS)
    {
        Serial.println("Receive mutex creation finished.");
    }

    // Initialize BLE
    BLEDevice::init("ESP32_Client_Alert_Data_Transmission"); // Using client MAC in name

    String bleMac = BLEDevice::getAddress().toString().c_str();
    Serial.println("BLE MAC: " + bleMac);
    
    // Attempt connection
    while (!connectToServer()) {
        Serial.println("Retrying in 5 seconds...");
        delay(2000);
    }

    // Set bigger MTU
    setMtuSize(pClient);

    // Setup reconstruction timer
    esp_timer_create_args_t reconstructionTimerArgs = {
        .callback = &powerReconstruction,
        .name = "reconstructionTimer"
    };
    esp_timer_create(&reconstructionTimerArgs, &reconstructionTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("Created reconstruction timer.");
    }
}

void loop() {
    if (!connected && disconnected) {
        Serial.println("Attempting reconnection...");
        if (connectToServer()) 
        {
            Serial.println("Reconnected successfully");
        }
        else
        {
            delay(2000);
        }
        
    }
        
    if (connected && (transferActive || readyForTransfer))
    {
        // Initialize data transfers
        if (readyForTransfer) 
        {
            Serial.println("Entering the transfer loop!");
            readyForTransfer = false;
            transferActive = true;
            receivedBytes = 0;
            buffersReceived = 0;
            request.command = 1;
            request.alertTimeStamp = serverAlertTimeStamp;
            sendTransferRequest();
        }
        // Request next chunk
        else if ((request.command == 2) && sendCommand)
        {
            sendCommand = false;
            sendTransferRequest();
        }
        // End past buffer transfer
        else if ((request.command == 3) && sendCommand)
        {
            sendCommand = false;
            sendTransferRequest();
        }
        // End all transfers
        else if ((request.command == 4) && sendCommand)
        {
            if (reconstructionTriggered)
            {
                Serial.println("[Reconstruction] Add new threshold.");
                reconstructionTriggered = false;
            }
            sendCommand = false;
            transferActive = false;
            sendTransferRequest();
            printPowerValues();
        }
    }

    delay(200); // Prevent watchdog triggers
}