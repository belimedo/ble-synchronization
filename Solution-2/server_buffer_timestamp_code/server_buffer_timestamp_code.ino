#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "../logged_data.h"

#define SAMPLING_FREQUENCY  (5) // Frekvencija samplovanja
#define MAX_BUFFER_SIZE     (256) // Velicina bafera bez gubitka podataka
#define TIME_COVERED        (MAX_BUFFER_SIZE / sizeof(uint8_t) / SAMPLING_FREQUENCY) // Vrijeme koje jedan bafer prenosi
#define TRUE_BUFFER_SIZE    (TIME_COVERED * sizeof(uint8_t) * SAMPLING_FREQUENCY ) // Velicina bafera koja ce se prenositi
#define NUMBER_OF_BUFFERS   (3) // Trenutni broj, a moglo bi se i izracunati na osnovu vremena koje donosi jedan bafer pa tako kontrolisati i broj bafera kao WANTED_TIME / TIME_COVERED
#define TIMEOUT_PERIOD      (1.f / SAMPLING_FREQUENCY)


typedef struct buffer_structure
{
    uint64_t time_stamp;
    uint8_t buffer_values[TRUE_BUFFER_SIZE];
} BUFFER;

BUFFER buffers[NUMBER_OF_BUFFERS]; 

// BLE Configuration
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TX_DATA_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (buffer data)
#define RX_CONTROL_UUID     "2c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Write (control)
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (timestamp)

// GPIO for debug output
const int redPin = 2;
const int bluePin = 22;
const int greenPin = 3;
int pinLayout[NUMBER_OF_BUFFERS] = {redPin, greenPin, bluePin};

// mutex init
SemaphoreHandle_t bufferMutex[NUMBER_OF_BUFFERS]; // One mutex per buffer
bool bufferLocked[NUMBER_OF_BUFFERS] = {false};   // Track lock status

// BLE Objects
BLEServer *pServer;
BLECharacteristic *pTxData;
BLECharacteristic *pRxControl;
BLECharacteristic *pTimestampChar;
bool deviceConnected = false;
bool firstRoundCompleted = false;
uint8_t currentWritingBufferIdx = 0; // Pokazuje koji je bafer trenutno aktivan za upisivanje
uint32_t sampleCounter = 0;
uint32_t currentSampleCounter = 0;
uint32_t bufferCounters[NUMBER_OF_BUFFERS] = {0};
uint64_t lastSampleTime = 0;
int activePin = pinLayout[currentWritingBufferIdx];
//int portMAX_DELAY = 0;

// Buffer transfer control
bool waitForAck = false;
uint8_t transferBufferIdx = 0;
uint16_t transferOffset = 0;
uint16_t maxSize = 0;
const uint16_t CHUNK_SIZE = 180; // Based on negotiated MTU of 247

// Timer handles
esp_timer_handle_t sampleTimer;
esp_timer_handle_t timestampTimer;

// Sampling callback
void sampleData(void* arg) {
    // bool bufferAdvanced = false;
    // Try to lock current buffer (non-blocking)
    if(xSemaphoreTake(bufferMutex[currentWritingBufferIdx], 0) == pdTRUE) 
    {
        // get time stamp  
        uint64_t now = esp_timer_get_time();
        // Get current buffer
        BUFFER* currentBuffer = &buffers[currentWritingBufferIdx];

        // Podijeli sa modulom TRUE_BUFFER_SIZE kad se uzima pun bafer pa da ide ispocetka
        bufferCounters[currentWritingBufferIdx] %= TRUE_BUFFER_SIZE;
        currentSampleCounter = bufferCounters[currentWritingBufferIdx];

        // Initialize timestamp on first sample
        if (currentSampleCounter == 0) {
            currentBuffer->time_stamp = now;
            bufferLocked[currentWritingBufferIdx] = false; // Reset lock flag ovo se ne koristi nigdje, malo halucinira
        }
        
        // Store data (circular buffer within LOGGED_DATA)
        currentBuffer->buffer_values[currentSampleCounter] = LOGGED_DATA[sampleCounter % LOGGED_DATA_SIZE];
        Serial.printf("Writting down buffer:%d[%d] = %d \n", currentWritingBufferIdx, currentSampleCounter, currentBuffer->buffer_values[currentSampleCounter]);
        sampleCounter++;
        bufferCounters[currentWritingBufferIdx]++;
        
        // Switch buffers when current one is full
        if (bufferCounters[currentWritingBufferIdx] >= TRUE_BUFFER_SIZE) {

            // bufferSampleCounter = 0; ne treba sada ovdje reset
            xSemaphoreGive(bufferMutex[currentWritingBufferIdx]); // Release current

            // Find next available buffer
            // Pored ovog koji je popunjen, probaj vidjeti ima li neki semafor slobodan naredni da taj postane iduci za upis
            uint8_t originalIdx = currentWritingBufferIdx;
            do {
                currentWritingBufferIdx = (currentWritingBufferIdx + 1) % NUMBER_OF_BUFFERS;
                if(currentWritingBufferIdx == originalIdx) {
                    Serial.println("ALL BUFFERS LOCKED! Dropping sample.");
                    return;
                }
            } while(xSemaphoreTake(bufferMutex[currentWritingBufferIdx], 0) != pdTRUE);

            // bufferAdvanced = true;
            digitalWrite(activePin, LOW); // Turn off current LED
            // currentWritingBufferIdx = (currentWritingBufferIdx + 1) % NUMBER_OF_BUFFERS; Visak, jer gore smo nasli bafer koji je prazan
            activePin = pinLayout[currentWritingBufferIdx];
            digitalWrite(activePin, HIGH); // Turn on new LED
            firstRoundCompleted = true;
            Serial.printf("Switching to buffer: %d\n", currentWritingBufferIdx);
        }
        // If we don't need to switch to the next buffer, release current buffer from a Mutex.
        // if(!bufferAdvanced) 
        // {
        // Release semaphore no matter what, as we are taking it either in if or in line 106
        xSemaphoreGive(bufferMutex[currentWritingBufferIdx]);
        // }
    }
    else 
    {
        Serial.printf("Buffer %d locked, trying to access other buffers.\n", currentWritingBufferIdx);
        // pronadji naredni koji slobodan
        uint8_t originalIdx = currentWritingBufferIdx;
        do {
            currentWritingBufferIdx = (currentWritingBufferIdx + 1) % NUMBER_OF_BUFFERS;
            if(currentWritingBufferIdx == originalIdx) {
                Serial.println("ALL BUFFERS LOCKED! Dropping sample.");
                sampleCounter++;
                return;
            }
        } while(xSemaphoreTake(bufferMutex[currentWritingBufferIdx], 0) != pdTRUE);
        bufferCounters[currentWritingBufferIdx] = 0; // resetuj bafer kaunter slobodnog bafera na 0
        xSemaphoreGive(bufferMutex[currentWritingBufferIdx]);
        Serial.println("New call.");
        sampleData(NULL);
        // sampleCounter++; // Nastavi brojati
    }
}

// Timestamp notification callback
void sendTimestamp(void* arg) {
    if (deviceConnected) {
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
        Serial.println(TRUE_BUFFER_SIZE);
        // Restart timstamp notifications on reconnect
        esp_timer_start_periodic(timestampTimer, 60 * 1000000);
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
        esp_timer_stop(timestampTimer);
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
                if (!firstRoundCompleted and currentWritingBufferIdx == 0){
                    Serial.println("Only one buffer filled");
                    transferBufferIdx = 0;
                    maxSize = bufferCounters[currentWritingBufferIdx];
                }
                else {
                    transferBufferIdx = (currentWritingBufferIdx == 0) ? NUMBER_OF_BUFFERS - 1 : currentWritingBufferIdx - 1; // Ovo uzima prethodni pun bafer, a ne trenutni
                    maxSize = TRUE_BUFFER_SIZE;
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

// Send next chunk of buffer data (only buffer_values)
void sendNextChunk() { // TODO: Ovdje uz poziv funkcije treba postaviti koliko se bajtova salje! 
    // When sending the message, we should block everything until the mutex is released 
    if(xSemaphoreTake(bufferMutex[transferBufferIdx], 2) == pdTRUE) {
        uint16_t chunkSize = 0;
        size_t bufferSize = maxSize * sizeof(uint8_t); // Only send buffer_values
        if (bufferSize > CHUNK_SIZE)
        {
            // Take a buffer 
            BUFFER *currentBuffer = &buffers[transferBufferIdx];
            
            // Calculate remaining data
            uint16_t remaining = bufferSize - transferOffset;
            if (remaining == 0) {
                Serial.println("Buffer transfer complete");
                xSemaphoreGive(bufferMutex[transferBufferIdx]);
                return;
            }

            // Prepare chunk
            chunkSize = min(remaining, CHUNK_SIZE);
            uint8_t *dataPtr = currentBuffer->buffer_values + transferOffset;
            
            // Send chunk
            pTxData->setValue(dataPtr, chunkSize);
            pTxData->notify();
            
            transferOffset += chunkSize;
            waitForAck = true;
        }
        else
        {
            // Take a buffer 
            BUFFER *currentBuffer = &buffers[transferBufferIdx];
            uint8_t *dataPtr = currentBuffer->buffer_values + transferOffset;
            
            // Send chunk
            chunkSize = bufferSize;
            pTxData->setValue(dataPtr, bufferSize);
            pTxData->notify();
            
            waitForAck = false;
            //transferOffset += chunkSize;
            
        }
        
        xSemaphoreGive(bufferMutex[transferBufferIdx]);
        
        Serial.printf("Sent chunk %d-%d of buffer %d\n", 
                    transferOffset - chunkSize, transferOffset - 1, transferBufferIdx);
    } 
    else {
        Serial.println("Failed to acquire buffer lock!");
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize LEDs
    for(int i = 0; i < NUMBER_OF_BUFFERS; i++) {
        pinMode(pinLayout[i], OUTPUT);
        digitalWrite(pinLayout[i], LOW);
    }
    digitalWrite(activePin, HIGH); // Active buffer LED on
    
    // Initialize buffers
    memset(buffers, 0, sizeof(buffers));
    memset(bufferCounters, 0, sizeof(bufferCounters));
    Serial.println("Cleared all buffers!");

    for(int i = 0; i < NUMBER_OF_BUFFERS; i++) {
        // kreiraj mutekse
        bufferMutex[i] = xSemaphoreCreateMutex();
        if(bufferMutex[i] == NULL) {
            Serial.println("Mutex creation failed!");
            while(1); // Critical failure
        }
    }
    
    // Initialize BLE
    BLEDevice::init("ESP32_Data_Logger_Semaphore");
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
    
    pService->start();
    BLEDevice::startAdvertising();
    
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
    
    Serial.println("BLE Data Logger Ready");
}

void loop() {
    delay(50); // Prevent watchdog reset
}