#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include "esp_timer.h"
#include "freertos/semphr.h"  // For semaphore
#include "logged_data.h"
#include "buffer_helper.h"

// BLE Configuration
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TX_DATA_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (buffer data)
#define RX_CONTROL_UUID     "2c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Write (control)
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (timestamp)

// GPIO Configuration
#define RX_RED_PIN          (2)
#define RX_BLUE_PIN         (22)
#define RX_GREEN_PIN        (3)
#define TX_RED_PIN          (34)
#define TX_BLUE_PIN         (32)
#define TX_GREEN_PIN        (25)
#define SEMAPHORE_PIN       (36)

int RXpinLayout[NUMBER_OF_BUFFERS] = {RX_RED_PIN, RX_GREEN_PIN, RX_BLUE_PIN};
int TXpinLayout[NUMBER_OF_BUFFERS] = {TX_RED_PIN, TX_GREEN_PIN, TX_BLUE_PIN};

// BLE Objects
BLEServer *pServer;
BLECharacteristic *pTxData;
BLECharacteristic *pRxControl;
BLECharacteristic *pTimestampChar;
bool deviceConnected = false;

// Buffer Management
SemaphoreHandle_t bufferSemaphore;  // Protects buffers during transfer
uint8_t currentBufferIdx = 0;
uint32_t sampleCounter = 0;
uint32_t bufferSampleCounter = 0;
uint64_t lastTimestampSent = 0;
int activeRXPin = RXpinLayout[currentBufferIdx];
int activeTXPin = TXpinLayout[currentBufferIdx];

// Transfer Control
bool waitForAck = false;
uint8_t transferBufferIdx = 0;
uint16_t transferOffset = 0;
const uint16_t CHUNK_SIZE = 180;

// Timer Handles
esp_timer_handle_t sampleTimer;
esp_timer_handle_t timestampTimer;

// Sampling callback with semaphore protection
void sampleData(void* arg) {
    BUFFER* currentBuffer = &buffers[currentBufferIdx];
    
    // Initialize timestamp on first sample
    if (bufferSampleCounter == 0) {
        currentBuffer->time_stamp = esp_timer_get_time();
    }
    // TODO: Ovo trenutno ne preskace sample-ove, mozda bi trebalo implementirati tako da ako nema mogucnost upisa
    // da preskoci ovaj sample, odnosno da imamo sampleCounter++
    // Try to get semaphore (non-blocking)
    if (xSemaphoreTake(bufferSemaphore, 0) == pdTRUE) {
        currentBuffer->buffer_values[bufferSampleCounter] = LOGGED_DATA[sampleCounter % LOGGED_DATA_SIZE];
        xSemaphoreGive(bufferSemaphore);
    } else {
        digitalWrite(SEMAPHORE_PIN, HIGH);  // Visualize semaphore contention
        return;  // Skip this sample if buffer is locked
    }
    
    sampleCounter++;
    bufferSampleCounter++;
    
    // Handle buffer switching
    if (bufferSampleCounter >= TRUE_BUFFER_SIZE) {
        bufferSampleCounter = 0;
        digitalWrite(activeRXPin, LOW);
        currentBufferIdx = (currentBufferIdx + 1) % NUMBER_OF_BUFFERS;
        activeRXPin = RXpinLayout[currentBufferIdx];
        digitalWrite(activeRXPin, HIGH);
        Serial.printf("Switched to buffer: %d\n", currentBufferIdx);
    }
    digitalWrite(SEMAPHORE_PIN, LOW);
}

// Timestamp notification callback
void sendTimestamp(void* arg) {
    if (deviceConnected) {
        uint64_t now = esp_timer_get_time();
        pTimestampChar->setValue((uint8_t *)&now, 8);
        pTimestampChar->notify();
        lastTimestampSent = now;
        Serial.printf("Timestamp sent: %llu µs\n", now);
    }
}

// Enhanced Control Callback with LED feedback
class ControlCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue().c_str();
        Serial.println("Control command: " + value);
        
        if (value == "REQUEST") {
            if (!waitForAck) {
                transferBufferIdx = (currentBufferIdx == 0) ? NUMBER_OF_BUFFERS - 1 : currentBufferIdx - 1;
                transferOffset = 0;
                digitalWrite(TXpinLayout[transferBufferIdx], HIGH);  // Activate TX LED
                sendNextChunk();
            }
        } 
        else if (value == "ACK") {
            waitForAck = false;
            sendNextChunk();
        }
    }
};

// Protected buffer transfer with semaphore
void sendNextChunk() {
    BUFFER *currentBuffer = &buffers[transferBufferIdx];
    size_t bufferSize = TRUE_BUFFER_SIZE * sizeof(uint8_t);
    
    if (xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) {
        uint16_t remaining = bufferSize - transferOffset;
        if (remaining == 0) {
            digitalWrite(TXpinLayout[transferBufferIdx], LOW);  // Turn off TX LED
            Serial.println("Buffer transfer complete");
            xSemaphoreGive(bufferSemaphore);
            return;
        }

        uint16_t chunkSize = min(remaining, CHUNK_SIZE);
        pTxData->setValue(currentBuffer->buffer_values + transferOffset, chunkSize);
        pTxData->notify();
        
        transferOffset += chunkSize;
        waitForAck = true;
        
        Serial.printf("Sent chunk %u-%u of buffer %u\n", 
                    transferOffset - chunkSize, transferOffset - 1, transferBufferIdx);
        xSemaphoreGive(bufferSemaphore);
    }
}

void setup_pins() {
    // Initialize RX LEDs
    for(int i = 0; i < NUMBER_OF_BUFFERS; i++) {
        pinMode(RXpinLayout[i], OUTPUT);
        digitalWrite(RXpinLayout[i], LOW);
    }
    digitalWrite(activeRXPin, HIGH);
    
    // Initialize TX LEDs
    for(int i = 0; i < NUMBER_OF_BUFFERS; i++) {
        pinMode(TXpinLayout[i], OUTPUT);
        digitalWrite(TXpinLayout[i], LOW);
    }
    
    // Initialize semaphore pin
    pinMode(SEMAPHORE_PIN, OUTPUT);
    digitalWrite(SEMAPHORE_PIN, LOW);
}

void setup() {
    Serial.begin(115200);
    
    // Create semaphore
    bufferSemaphore = xSemaphoreCreateMutex();
    
    // Initialize hardware
    setup_pins();
    memset(buffers, 0, sizeof(buffers));
    
    // BLE Initialization
    BLEDevice::init("ESP32_Data_Logger_Semaphore");
    BLEDevice::setMTU(247);
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Characteristics setup (same as before)
    pTxData = pService->createCharacteristic(TX_DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pTxData->addDescriptor(new BLE2902());
    BLE2901* pTxDataDesc = new BLE2901();
    pTxDataDesc->setDescription("Buffer values (uint8_t array)");
    pTimestampChar->addDescriptor(pTxDataDesc);
    
    pTimestampChar = pService->createCharacteristic(TIMESTAMP_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pTimestampChar->addDescriptor(new BLE2902());
    BLE2901* pTimestampCharDesc = new BLE2901();
    pTimestampCharDesc->setDescription("Timestamp (µs since boot)");
    pTimestampChar->addDescriptor(pTimestampCharDesc);
    
    pRxControl = pService->createCharacteristic(RX_CONTROL_UUID, BLECharacteristic::PROPERTY_WRITE);
    pRxControl->setCallbacks(new ControlCallback());
    BLE2901* pRxDesc = new BLE2901();
    pRxDesc->setDescription("Control: REQUEST/ACK");
    pRxControl->addDescriptor(pRxDesc);
    
    pService->start();
    BLEDevice::startAdvertising();
    
    // Timer setup
    esp_timer_create_args_t sampleTimerArgs = { .callback = &sampleData, .name = "dataSampler" };
    esp_timer_create(&sampleTimerArgs, &sampleTimer);
    
    esp_timer_create_args_t tsTimerArgs = { .callback = &sendTimestamp, .name = "timestampNotifier" };
    esp_timer_create(&tsTimerArgs, &timestampTimer);
    
    float samplingIntervalUs = (1.f / SAMPLING_FREQUENCY) * 1000000;
    esp_timer_start_periodic(sampleTimer, samplingIntervalUs);
    esp_timer_start_periodic(timestampTimer, 60 * 1000000);
    
    Serial.println("BLE Data Logger with Semaphore Ready");
}

void loop() {
    delay(500);
}