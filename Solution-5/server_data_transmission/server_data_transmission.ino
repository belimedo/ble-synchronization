#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "esp_bt.h"
//#include "esp_bt_main.h"
//#include "esp_gap_ble_api.h"
#include "../logged_data.h"
#include "../communication_structures.h"

// BLE Configuration
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914a"
#define TX_DATA_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (buffer data)
#define RX_CONTROL_UUID     "2c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Write (control)
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (timestamp)
#define LATENCY_UUID        "4c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify Write (latency)
#define ALERT_MEAS_UUID     "ac95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (incorrect data)

// store buffer variables
STORE_BUFFER storeBuffers[NUMBER_OF_STORE_BUFFERS];
TRANSFER_BUFFER transferBuffer;

uint8_t currentStoreBufferIdx = 0;
uint8_t currentSendingBufferIdx;

// mutex init
SemaphoreHandle_t storeBufferMutex[NUMBER_OF_STORE_BUFFERS]; // One mutex per store buffer
SemaphoreHandle_t sendBufferMutex[NUMBER_OF_SEND_BUFFERS]; // One mutex per send buffer

// GPIO for debug output
const int redPin = 2;
const int bluePin = 22;
const int greenPin = 3;
int pinLayout[NUMBER_OF_STORE_BUFFERS] = {redPin, greenPin, bluePin};
int activePin = pinLayout[currentStoreBufferIdx];


// BLE Objects
BLEServer *pServer;
BLECharacteristic *pTxData;
BLECharacteristic *pRxControl;
BLECharacteristic *pTimestampChar;
BLECharacteristic *pLatencyControl; // For latency characteristic
BLECharacteristic *pAlertMeasurementChar; // For incorrect measurement


bool deviceConnected = false;
bool advertising = false;
bool latencyTestMode = true;
bool startLatencyTest = false; // Vrijednost koja pokazuje da li pocinjemo ili ne sa mjerenjem vremena, podesava se na true sa prvim pingom
uint8_t latencyCounter = 0; // TODO: Obrisati

uint32_t sampleCounter = 0;
uint32_t currentSampleCounter = 0;
uint32_t bufferCounters[NUMBER_OF_STORE_BUFFERS] = {0};
uint64_t lastSampleTime = 0;


// Buffer transfer control
bool waitForAck = false;
uint8_t transferBufferIdx = 0;
uint16_t transferOffset = 0;
uint16_t maxSize = 0;

// Error data global data
#define THRESHOLD_VALUE         (250) // 0xFA

const uint16_t CHUNK_SIZE = 180; // Based on negotiated MTU of 247

// Debug handling stuff
bool DEBUG_PRINTS = false;


// Timer handles
esp_timer_handle_t sampleTimer;
esp_timer_handle_t timestampTimer;

// Sampling callback
void sampleData(void* arg) {
    // declare variable where data should be stored at
    uint8_t sampledData;
    // Try to lock current buffer (non-blocking)
    if(xSemaphoreTake(storeBufferMutex[currentStoreBufferIdx], 0) == pdTRUE) 
    {
        // get time stamp  
        uint64_t now = esp_timer_get_time();
        // Get current buffer
        STORE_BUFFER *currentBuffer = &storeBuffers[currentStoreBufferIdx];

        // Podijeli sa modulom STORE_BUFFER_SIZE kad se uzima pun bafer pa da ide ispocetka
        currentBuffer->sampleIdx %= STORE_BUFFER_SIZE;
        currentSampleCounter = currentBuffer->sampleIdx;

        // Initialize timestamp on first sample
        if (currentSampleCounter == 0) {
            currentBuffer->startTimeStamp = now;
        }
        
        // Store data (circular buffer within LOGGED_DATA)
        sampledData = LOGGED_DATA[sampleCounter % LOGGED_DATA_SIZE]; // This can be read from ADC
        
        currentBuffer->buffer[currentSampleCounter] = sampledData;
        if (DEBUG_PRINTS)
        {
            Serial.printf("Writting down buffer:%d[%d] = %d \n", currentStoreBufferIdx, currentSampleCounter, currentBuffer->buffer[currentSampleCounter]);
        }
        // increment counters
        sampleCounter++;
        currentBuffer->sampleIdx++;
        
        // Switch buffers when current one is full
        if (currentBuffer->sampleIdx >= STORE_BUFFER_SIZE) {

            // bufferSampleCounter = 0; ne treba sada ovdje reset
            xSemaphoreGive(storeBufferMutex[currentStoreBufferIdx]); // Release current

            // Find next available buffer
            // Pored ovog koji je popunjen, probaj vidjeti ima li neki semafor slobodan naredni da taj postane iduci za upis
            uint8_t originalIdx = currentStoreBufferIdx;
            do {
                currentStoreBufferIdx = (currentStoreBufferIdx + 1) % NUMBER_OF_STORE_BUFFERS;
                if(currentStoreBufferIdx == originalIdx) {
                    Serial.println("ALL BUFFERS LOCKED! Dropping sample.");
                    return;
                }
            } while(xSemaphoreTake(storeBufferMutex[currentStoreBufferIdx], 0) != pdTRUE);

            digitalWrite(activePin, LOW); // Turn off current LED
            activePin = pinLayout[currentStoreBufferIdx];
            digitalWrite(activePin, HIGH); // Turn on new LED
            Serial.printf("Switching to buffer: %d\n", currentStoreBufferIdx);
        }
        xSemaphoreGive(storeBufferMutex[currentStoreBufferIdx]);
        
        if (deviceConnected && sampledData >= THRESHOLD_VALUE)
        {
            Serial.println("Alerting the client that server discovered abnormality.");
            Serial.printf("Time of detection: %llu\n", now);
            pAlertMeasurementChar->setValue((uint8_t *)&now, 8);
            pAlertMeasurementChar->notify();
        }
    }
    else 
    {
        Serial.printf("Buffer %d locked, trying to access other buffers.\n", currentStoreBufferIdx);
        // pronadji naredni koji slobodan
        uint8_t originalIdx = currentStoreBufferIdx;
        do {
            currentStoreBufferIdx = (currentStoreBufferIdx + 1) % NUMBER_OF_STORE_BUFFERS;
            if(currentStoreBufferIdx == originalIdx) {
                Serial.println("ALL BUFFERS LOCKED! Dropping sample.");
                sampleCounter++;
                return;
            }
        } while(xSemaphoreTake(storeBufferMutex[currentStoreBufferIdx], 0) != pdTRUE);
        
        storeBuffers[currentStoreBufferIdx].sampleIdx = 0; // resetuj bafer kaunter slobodnog bafera na 0
        xSemaphoreGive(storeBufferMutex[currentStoreBufferIdx]);
        if (DEBUG_PRINTS)
        {
            Serial.println("Recursive call of .");
        }
        sampleData(NULL);
    }
}

// Timestamp notification callback
void sendTimestamp(void* arg) {
    if (deviceConnected and !latencyTestMode) {
        uint64_t now = esp_timer_get_time();
        pTimestampChar->setValue((uint8_t *)&now, 8);
        pTimestampChar->notify();
        Serial.printf("Timestamp sent: %llu µs\n", now);
    }
}

// BLE Server Callbacks
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client connected");
        Serial.println("Buffer size:");
        Serial.println(STORE_BUFFER_SIZE);
        // Restart timstamp notifications on reconnect
        esp_timer_start_periodic(timestampTimer, 60 * 1000000);
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
        esp_timer_stop(timestampTimer);

        // Critical fixes:
        BLEDevice::startAdvertising();
        advertising = true;
        delay(500); // Allow advertising to start
    }
};

// Control Characteristic Callback
class ControlCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue().c_str();
        Serial.println("On write is active");
        Serial.println(value);
        
        if (value == "REQ") {
            if (!waitForAck) {
                // Start new transfer with the most recent complete buffer
                if (currentStoreBufferIdx == 0){
                    Serial.println("Only one buffer filled");
                    transferBufferIdx = 0;
                    maxSize = storeBuffers[currentStoreBufferIdx].sampleIdx;
                }
                else {
                    transferBufferIdx = (currentStoreBufferIdx == 0) ? NUMBER_OF_STORE_BUFFERS - 1 : currentStoreBufferIdx - 1; // Ovo uzima prethodni pun bafer, a ne trenutni
                    maxSize = STORE_BUFFER_SIZE;
                }
                // TODO: Ovdje popuniti kako se racuna vrijeme
                transferOffset = 0;
                sendNextChunk();
            }
        } 
        else if (value == "ACK") {
            // Client confirms receipt; send next chunk
            waitForAck = false;
            sendNextChunk();
        }
    }
};


class ControlCallbacksLatency: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue();
        Serial.printf("Received value %s with length %d.\n", value, value.length());

        if (startLatencyTest && value.length() == sizeof(uint64_t)) // value.length() == sizeof(uint64_t)
        {
            uint64_t clientSendTime = *((uint64_t*)value.c_str());
            //uint64_t clientSendTime = strtoull(value.c_str(), NULL, 10);
            pLatencyControl->setValue((uint8_t*)&clientSendTime, sizeof(clientSendTime));
            pLatencyControl->notify();
            Serial.printf("Received [%d] timestamp: %llu\n", ++latencyCounter, clientSendTime);
        }
        else if (value == "PING") {
            // Immediately echo back with server's receive time
            latencyCounter = 0;
            startLatencyTest = true;
            uint64_t clientSendTime = 0;
            pLatencyControl->setValue((uint8_t*)&clientSendTime, sizeof(clientSendTime));
            pLatencyControl->notify();
            Serial.println("Starting latency test.");
        }
        else if (value == "START_TIMESTAMPS") {
            latencyTestMode = false;
            startLatencyTest = false;
            Serial.println("Switching to timestamp mode");
        }
    }
};

// Send next chunk of buffer data (only buffer_values)
void sendNextChunk() { // TODO: Ovdje uz poziv funkcije treba postaviti koliko se bajtova salje! 
    // When sending the message, we should block everything until the mutex is released 
    if(xSemaphoreTake(sendBufferMutex[transferBufferIdx], 2) == pdTRUE) {
        uint16_t chunkSize = 0;
        size_t bufferSize = maxSize * sizeof(uint8_t); // Only send buffer_values
        if (bufferSize > CHUNK_SIZE)
        {
            // Take a buffer 
            // BUFFER *currentBuffer = transferBuffer[transferBufferIdx];
            
            // Calculate remaining data
            uint16_t remaining = bufferSize - transferOffset;
            if (remaining == 0) {
                Serial.println("Buffer transfer complete");
                xSemaphoreGive(sendBufferMutex[transferBufferIdx]);
                return;
            }

            // Prepare chunk
            chunkSize = min(remaining, CHUNK_SIZE);
            uint8_t *dataPtr = transferBuffer.previousSamples.dataBufferPtr + transferOffset;
            
            // Send chunk
            pTxData->setValue(dataPtr, chunkSize);
            pTxData->notify();
            
            transferOffset += chunkSize;
            waitForAck = true;
        }
        else
        {
            // TODO: Staviti send buffer ovdje! 
            // Take a buffer 
            uint8_t *dataPtr = transferBuffer.previousSamples.dataBufferPtr + transferOffset;
            
            // Send chunk
            chunkSize = bufferSize;
            pTxData->setValue(dataPtr, bufferSize);
            pTxData->notify();
            
            waitForAck = false;
            //transferOffset += chunkSize;
            
        }
        
        xSemaphoreGive(sendBufferMutex[transferBufferIdx]);
        
        Serial.printf("Sent chunk %d-%d of buffer %d\n", 
                    transferOffset - chunkSize, transferOffset - 1, transferBufferIdx);
    } 
    else {
        Serial.println("Failed to acquire buffer lock!");
    }
}

void setup() {
    // setup serial for writing
    Serial.begin(115200);
    
    // Initialize LEDs
    for(int i = 0; i < NUMBER_OF_STORE_BUFFERS; i++) {
        pinMode(pinLayout[i], OUTPUT);
        digitalWrite(pinLayout[i], LOW);
    }
    digitalWrite(activePin, HIGH); // Active buffer LED on
    
    // Initialize buffers
    memset(storeBuffers, 0, sizeof(storeBuffers)); // buffers for storing samples
    memset(&transferBuffer, 0, sizeof(transferBuffer)); // buffer for sending out time window
    currentStoreBufferIdx = 0;
    currentSendingBufferIdx = 0;
    Serial.println("Cleared all buffers and indices!");

    for(int i = 0; i < NUMBER_OF_STORE_BUFFERS; i++) {
        // kreiraj mutekse
        storeBufferMutex[i] = xSemaphoreCreateMutex();
        if(storeBufferMutex[i] == NULL) {
            Serial.println("Store mutex creation failed!");
            while(1); // Critical failure
        }
    }

    for(int i = 0; i < NUMBER_OF_SEND_BUFFERS; i++) {
        // kreiraj mutekse
        sendBufferMutex[i] = xSemaphoreCreateMutex();
        if(sendBufferMutex[i] == NULL) {
            Serial.println("Send mutex creation failed!");
            while(1); // Critical failure
        }
    }

    // esp_err_t ret = esp_ble_set_scan_duplicate_cache_size(200);
    // if (ret != ESP_OK) {
    //     Serial.printf("Failed to increase BLE cache: %d\n", ret);
    // }

    // Initialize BLE
    BLEDevice::init("ESP32_Server_Data_Transmission");
    BLEDevice::setMTU(247); // Request larger MTU
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // TX Data Characteristic (Notify) - For buffer values
    pTxData = pService->createCharacteristic(
        TX_DATA_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxData->addDescriptor(new BLE2902());
    // Add description
    BLE2901* pTxDesc = new BLE2901();
    pTxDesc->setDescription("Buffer values (uint8_t array)");
    pTxData->addDescriptor(pTxDesc);
    
    // Timestamp Characteristic (Notify)
    pTimestampChar = pService->createCharacteristic(
        TIMESTAMP_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTimestampChar->addDescriptor(new BLE2902());
    // Add description
    BLE2901* pTsDesc = new BLE2901();
    pTsDesc->setDescription("Timestamp (µs since boot)");
    pTimestampChar->addDescriptor(pTsDesc);

    // Alert Characteristic (Notify)
    pAlertMeasurementChar = pService->createCharacteristic(
        ALERT_MEAS_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAlertMeasurementChar->addDescriptor(new BLE2902());
    // Add description
    BLE2901* pAlertDesc = new BLE2901();
    pAlertDesc->setDescription("Alert timestamp");
    pAlertMeasurementChar->addDescriptor(pAlertDesc);

    // Latency Control Characteristic (write and notify)
    pLatencyControl = pService->createCharacteristic(
        LATENCY_UUID,
        BLECharacteristic::PROPERTY_WRITE | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pLatencyControl->setCallbacks(new ControlCallbacksLatency());
    pLatencyControl->addDescriptor(new BLE2902());
    BLE2901* pLatencyDesc = new BLE2901();
    pLatencyDesc->setDescription("Latency control.");
    pLatencyControl->addDescriptor(pLatencyDesc);
    
    // RX Control Characteristic (Write)
    pRxControl = pService->createCharacteristic(
        RX_CONTROL_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxControl->setCallbacks(new ControlCallback());
    // Add description
    BLE2901* pRxDesc = new BLE2901();
    pRxDesc->setDescription("Control: REQUEST/ACK");
    pRxControl->addDescriptor(pRxDesc);



    // BLE MAC
    String bleMac = BLEDevice::getAddress().toString();
    Serial.println("BLE MAC Address: " + bleMac);

    // After creating all characteristics
    Serial.println("Server characteristics created:");
    Serial.printf("- TX Data: %s\n", pTxData->getUUID().toString().c_str());
    Serial.printf("- RX Control: %s\n", pRxControl->getUUID().toString().c_str());
    Serial.printf("- Timestamp: %s\n", pTimestampChar->getUUID().toString().c_str());
    Serial.printf("- Latency: %s\n", pLatencyControl->getUUID().toString().c_str());
    Serial.printf("- Alert: %s\n", pAlertMeasurementChar->getUUID().toString().c_str());

    
    pService->start();
    // Configure advertising properly
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Helps with iPhone connections
    BLEDevice::startAdvertising();
    advertising = true;
    Serial.println("Server ready for connections");
    
    // Setup sampling timer
    esp_timer_create_args_t sampleTimerArgs = {
        .callback = &sampleData,
        .name = "dataSampler"
    };
    esp_timer_create(&sampleTimerArgs, &sampleTimer);
    
    // Setup timestamp timer
    esp_timer_create_args_t tsTimerArgs = {
        .callback = &sendTimestamp,
        .name = "timestampNotifier"
    };
    esp_timer_create(&tsTimerArgs, &timestampTimer);
    
    // Start timers
    float samplingIntervalUs = (1.f / SAMPLING_FREQUENCY) * 1000000;
    esp_timer_start_periodic(sampleTimer, samplingIntervalUs);
    Serial.println("Started sampling timer with sampling interval:");
    Serial.println(samplingIntervalUs);
    // esp_timer_start_periodic(timestampTimer, 60 * 1000000); // 60 seconds
    
    Serial.println("BLE alert data Ready");
}

void loop() {
    if (!deviceConnected && !advertising) {
        BLEDevice::startAdvertising();
        advertising = true;
        Serial.println("Restarting advertising");
    }
    delay(50); // Prevent watchdog reset
}