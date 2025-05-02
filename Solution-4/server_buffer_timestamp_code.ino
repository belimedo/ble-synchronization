#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include "esp_timer.h"
#include "logged_data.h"
#include "buffer_structure.h"

#define SAMPLING_FREQUENCY  (10) // Frekvencija samplovanja
#define MAX_BUFFER_SIZE     (256) // Velicina bafera bez gubitka podataka
#define TIME_COVERED        (MAX_BUFFER_SIZE / sizeof(uint8_t) / SAMPLING_FREQUENCY) // Vrijeme koje jedan bafer prenosi
#define TRUE_BUFFER_SIZE    (TIME_COVERED * sizeof(uint8_t) * SAMPLING_FREQUENCY ) // Velicina bafera koja ce se prenositi
#define NUMBER_OF_BUFFERS   (3) // Trenutni broj, a moglo bi se i izracunati na osnovu vremena koje donosi jedan bafer pa tako kontrolisati i broj bafera kao WANTED_TIME / TIME_COVERED


// BLE Configuration
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TX_DATA_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (buffer data)
#define RX_CONTROL_UUID     "2c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Write (control)
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (timestamp)

// GPIO for debug output
const int redPin = 2;
const int bluePin = 22;
const int greenPin = 3;
int pinLayout[NUMBER_OF_BUFFERS] = {redPin, bluePin, greenPin};

// BLE Objects
BLEServer *pServer;
BLECharacteristic *pTxData;
BLECharacteristic *pRxControl;
BLECharacteristic *pTimestampChar;
bool deviceConnected = false;
uint8_t currentBufferIdx = 0;
uint32_t sampleCounter = 0;
uint32_t bufferSampleCounter = 0;
int activePin = pinLayout[currentBufferIdx];

// Buffer transfer control
bool waitForAck = false;
uint8_t transferBufferIdx = 0;
uint16_t transferOffset = 0;
const uint16_t CHUNK_SIZE = 180; // Based on negotiated MTU of 247

// Timer handles
esp_timer_handle_t sampleTimer;
esp_timer_handle_t timestampTimer;

// Sampling callback
void sampleData(void* arg) {    
    // Get current buffer
    BUFFER* currentBuffer = &buffers[currentBufferIdx];
    
    // Initialize timestamp on first sample
    if (bufferSampleCounter == 0) {
        currentBuffer->time_stamp = esp_timer_get_time();
    }
    
    // Store data (circular buffer within LOGGED_DATA)
    currentBuffer->buffer_values[bufferSampleCounter] = LOGGED_DATA[sampleCounter % LOGGED_DATA_SIZE];
    sampleCounter++;
    bufferSampleCounter++;
    
    // Switch buffers when current one is full
    if (bufferSampleCounter >= TRUE_BUFFER_SIZE) {
        bufferSampleCounter = 0;
        digitalWrite(activePin, LOW); // Turn off current LED
        currentBufferIdx = (currentBufferIdx + 1) % NUMBER_OF_BUFFERS;
        activePin = pinLayout[currentBufferIdx];
        Serial.printf("Switching to buffer: %d\n", currentBufferIdx);
        digitalWrite(activePin, HIGH); // Turn on new LED
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
        Serial.println("Buffer size:  %llu", TRUE_BUFFER_SIZE);
        // Restart timestamp notifications on reconnect
        Serial.println("Starting timestamp timer.");
        esp_timer_start_periodic(timestampTimer, 60 * 1000000);
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected");
        Serial.println("Stopping timestamp timer.");
        esp_timer_stop(timestampTimer);
    }
};

// Control Characteristic Callback
class ControlCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String value = pChar->getValue().c_str();
        Serial.println("Control callback for write characteristic.");
        Serial.println("String value: %s", value);
        
        if (value == "REQ") {
            if (!waitForAck) {
                // Start new transfer with the most recent complete buffer
                transferBufferIdx = (currentBufferIdx == 0) ? NUMBER_OF_BUFFERS - 1 : currentBufferIdx - 1;
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
void sendNextChunk() {
    BUFFER *currentBuffer = &buffers[transferBufferIdx];
    size_t bufferSize = TRUE_BUFFER_SIZE * sizeof(uint8_t); // Only send buffer_values
    
    // Calculate remaining data
    uint16_t remaining = bufferSize - transferOffset;
    if (remaining == 0) {
        Serial.println("Buffer transfer complete.");
        return;
    }

    // Prepare chunk
    uint16_t chunkSize = min(remaining, CHUNK_SIZE);
    uint8_t *dataPtr = currentBuffer->buffer_values + transferOffset;
    
    // Send chunk
    pTxData->setValue(dataPtr, chunkSize);
    pTxData->notify();
    
    transferOffset += chunkSize;
    waitForAck = true;
    
    Serial.printf("Sent chunk %u-%u of buffer %u\n", 
                 transferOffset - chunkSize, transferOffset - 1, transferBufferIdx);
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
    
    // Initialize BLE
    BLEDevice::init("ESP32_Data_Logger");
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
    pRxDesc->setDescription("Control: REQ/ACK");
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
    Serial.println("Starting sampling timer. Sampling interval: %f\n",samplingIntervalUs);
    Serial.println(samplingIntervalUs);
    // esp_timer_start_periodic(timestampTimer, 60 * 1000000); // 60 seconds
    
    Serial.println("BLE Data Logger Ready");
}

void loop() {
    delay(500); // Prevent watchdog reset
}