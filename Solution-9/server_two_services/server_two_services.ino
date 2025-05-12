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
// #include "../logged_data.h"
#include "../logged_data_2.h"
#include "../communication_structures.h"
#include "../constants.h"

// Error data global data
#define THRESHOLD_VALUE         (255) // 0xFF

// store buffer variables
STORE_BUFFER storeBuffers[NUMBER_OF_STORE_BUFFERS];
sendBuffer bufferToSend;
sendBuffer buffersToSend[NUMBER_OF_SEND_BUFFERS];

// Debug handling stuff
bool DEBUG_PRINTS = true;

uint8_t currentStoreBufferIdx = 0;
uint8_t currentSendingBufferIdx = 0;

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
// BLECharacteristic *pLatencyControl; // For latency characteristic
BLECharacteristic *pAlertMeasurementChar; // For incorrect measurement


bool deviceConnected = false;
bool advertising = false;
// bool latencyTestMode = true;
// bool startLatencyTest = false; // Vrijednost koja pokazuje da li pocinjemo ili ne sa mjerenjem vremena, podesava se na true sa prvim pingom
bool sendAlerts = false; // Vrijednost koja pokazuje da li pocinjemo ili ne sa mjerenjem vremena, podesava se na true sa prvim pingom
// uint8_t latencyCounter = 0; // TODO: Obrisati

uint32_t sampleCounter = 0;
uint32_t currentSampleCounter = 0;
uint32_t bufferCounters[NUMBER_OF_STORE_BUFFERS] = {0};
uint64_t lastSampleTime = 0;

// Globalna promjenljiva za alert indeks
// TODO: Dodati nekakav semafor za ovo.
int alertEventIdx; // TODO: transferBufferIdx isto treba iskoristiti tj dodati nesto sto bi se zvalo alertBufferIdx
uint64_t alertTimeStamp; // Za future.

// Buffer transfer control
bool waitForAck = false;
int transferBufferIdx = 0;
uint16_t transferOffset = 0;
uint16_t maxSize = 0;

// data transfer values
bool transferInProgress = false;
uint16_t currentChunk = 0;
uint16_t totalChunks = 0;
// int sendBufferIdx;


uint16_t CHUNK_SIZE = DEFAULT_CHUNK_SIZE; // Default chunk size. Assign new value according to the MTU size (MTU - 3)

// Timer handles
float samplingIntervalUs = (1.f / SAMPLING_FREQUENCY) * 1000000;
esp_timer_handle_t sampleTimer;
esp_timer_handle_t timestampTimer;
esp_timer_handle_t futureBufferTimer;


void printHexValues()
{
    Serial.println("Printing future hex values:");
    for (int sendingBufferIdx = 0; sendingBufferIdx < NUMBER_OF_SEND_BUFFERS; sendingBufferIdx++)
    {
        Serial.printf("Printing %s buffer.", sendingBufferIdx % 2 == 0 ? "past" : "future");
        if(xSemaphoreTake(sendBufferMutex[sendingBufferIdx], 50) == pdTRUE) // busy wait 50 ms
        {
            Serial.printf("Start time of the data: %llu\n Data samples:", buffersToSend[sendingBufferIdx].timeStamp);
            for (int i = 0; i < SEND_BUFFER_SIZE; i++)
            {
                if ((i % SAMPLING_FREQUENCY) == 0)
                {
                    Serial.printf("\n");
                }
                Serial.printf("%d: 0x%2x, ", i, buffersToSend[sendingBufferIdx].dataBufferPtr[i]);
            }
            Serial.printf("\n");
            xSemaphoreGive(sendBufferMutex[sendingBufferIdx]);
        }
        else
        {
            Serial.println("Couldn't access lock in print hex values.");
        }
    }
}


// future buffer callback
void fillFutureBuffer(void* arg)
{
    if(xSemaphoreTake(storeBufferMutex[transferBufferIdx], 50) == pdTRUE) // busy wait 50 ms
    {
        Serial.printf("[Future buffer] Alert detected at %d idx, value:0x%2x\n", alertEventIdx, storeBuffers[transferBufferIdx].buffer[alertEventIdx]);
        Serial.printf("[Future buffer] 2 values before and after alert:\n0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n", 
            storeBuffers[transferBufferIdx].buffer[alertEventIdx - 2],storeBuffers[transferBufferIdx].buffer[alertEventIdx - 1],storeBuffers[transferBufferIdx].buffer[alertEventIdx],
            storeBuffers[transferBufferIdx].buffer[(alertEventIdx + 1) % STORE_BUFFER_SIZE], storeBuffers[transferBufferIdx].buffer[(alertEventIdx + 2) % STORE_BUFFER_SIZE]); // Moduo je za memory overflow
        int endBufferIdx = alertEventIdx + SEND_BUFFER_SIZE;
        // Ako se ovo dogodi, to znaci da nam treba jos jedan buffer
        if (endBufferIdx > STORE_BUFFER_SIZE)
        {
            Serial.println("[Future buffer] Data after alert is in two buffers.");
            // Razlika izmedju endBufferIdx i STORE_BUFFER_SIZE ce nam dati koliko treba narednog da se uzme a trenutnog je SEND_BUFFER_SIZE - razlika
            int overheadIndices = endBufferIdx - STORE_BUFFER_SIZE; // 153 - 150 = 3; Znaci treba nam do idx 150 (ne racunajuci njega) onda nam treba 150, 151, 152 (3 komada)
            int firstBufferSize = SEND_BUFFER_SIZE - overheadIndices;
            Serial.printf("[Future buffer] Copying first batch of %d indices.\n", firstBufferSize);
            if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE)
            {
                buffersToSend[currentSendingBufferIdx].timeStamp = alertTimeStamp;
                memcpy(buffersToSend[currentSendingBufferIdx].dataBufferPtr, &storeBuffers[transferBufferIdx].buffer[alertEventIdx], firstBufferSize);
                xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
            }
            else
            {
                Serial.println("Failed to fetch send buffer mutex.");
            }
            // Calculate next transfer buffer idx
            int nextTransferBufferIdx = transferBufferIdx == (NUMBER_OF_STORE_BUFFERS - 1) ? 0 : transferBufferIdx + 1; // Ako je transferBuffer 2, tada je iduci 0
            
            if(xSemaphoreTake(storeBufferMutex[nextTransferBufferIdx], 50) == pdTRUE) // busy wait 50 ms
            {
                Serial.printf("[Future buffer] Copying second batch of %d indices.\n", overheadIndices);
                if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE)
                {
                    memcpy(&buffersToSend[currentSendingBufferIdx].dataBufferPtr[firstBufferSize], &storeBuffers[nextTransferBufferIdx].buffer[0], overheadIndices);
                    xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                }
                else
                {
                    Serial.println("Failed to fetch send buffer mutex.");
                }    
                xSemaphoreGive(storeBufferMutex[nextTransferBufferIdx]); // Release current
            }
            else
            {
                Serial.printf("[Future buffer] Problem with allocating store buffer %d", nextTransferBufferIdx);
            }        
        }
        else
        {
            Serial.println("[Future buffer] Data after alert is in one buffer.");
            if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE)
            {
                buffersToSend[currentSendingBufferIdx].timeStamp = alertTimeStamp;
                memcpy(buffersToSend[currentSendingBufferIdx].dataBufferPtr, &storeBuffers[transferBufferIdx].buffer[alertEventIdx], SEND_BUFFER_SIZE);
                xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
            }
            else
            {
                Serial.printf("Failed to fetch send buffer mutex %d.\n", currentSendingBufferIdx);
            }
        }
        xSemaphoreGive(storeBufferMutex[transferBufferIdx]); // Release current
        //printHexValues();
        //printHexValues(); // TODO: Ovdje notify samo uraditi. sendChunk();
        sendChunk();
    }
    else
    {
        Serial.printf("[Future buffer] Problem with allocating store buffer %d", transferBufferIdx);
    }        
}


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
        // if (DEBUG_PRINTS)
        // {
        //     Serial.printf("Writting down buffer:%d[%d] = %d \n", currentStoreBufferIdx, currentSampleCounter, currentBuffer->buffer[currentSampleCounter]);
        // }
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
            Serial.printf("[Sample sync] Switching to buffer: %d\n", currentStoreBufferIdx);
        }
        xSemaphoreGive(storeBufferMutex[currentStoreBufferIdx]);
        
        // Trigger alert characteristic
        // Alerts cannot be triggered if current alert is being processed
        if (sendAlerts && deviceConnected && sampledData >= THRESHOLD_VALUE) // TODO: Mozda bih mogao samo sendAlerts onemoguciti dok se ne zavrsi proces
        {
            Serial.println("[Alert sync] Alerting the client that server discovered abnormality.");
            Serial.printf("[Alert sync] Time of detection: %llu\n", now);
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
        } while(xSemaphoreTake(storeBufferMutex[currentStoreBufferIdx], 10) != pdTRUE);
        
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
    if (deviceConnected) {
        uint64_t now = esp_timer_get_time();
        pTimestampChar->setValue((uint8_t *)&now, 8);
        pTimestampChar->notify();
        Serial.printf("Timestamp sent: %llu µs\n", now);
        sendAlerts = true;
    }
}

// BLE Server Callbacks
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client connected");
        Serial.println("Buffer size:");
        Serial.printf("[Server] MTU size: %d\n", pServer->getPeerMTU(pServer->getConnId()));
        CHUNK_SIZE =  pServer->getPeerMTU(pServer->getConnId()) - 3;
        Serial.printf("[Server] Max chunk size: %d\n", CHUNK_SIZE);
        Serial.println("Store buffer size:");
        Serial.println(STORE_BUFFER_SIZE);
        // Restart timstamp notifications on reconnect
        esp_timer_start_periodic(timestampTimer, 60 * 1000000);
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        sendAlerts = false;
        CHUNK_SIZE = DEFAULT_CHUNK_SIZE;
        Serial.println("Client disconnected");
        esp_timer_stop(timestampTimer);

        // Critical fixes:
        BLEDevice::startAdvertising();
        advertising = true;
        delay(500); // Allow advertising to start
    }

    void onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* desc) 
    {
        Serial.printf("New MTU: %d\n", pServer->getPeerMTU(pServer->getConnId()));
        CHUNK_SIZE =  pServer->getPeerMTU(pServer->getConnId()) - 3;
        Serial.printf("[Server] Max chunk size: %d\n", CHUNK_SIZE);
    }
};

class RxControlCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
        Serial.println("Received message!\nDisabeling alerts as well.");
        sendAlerts = false;
        String value = pCharacteristic->getValue();
        DATA_REQUEST request;
        // cast const char* to DATA_REQUEST struct
        memcpy(&request, (uint8_t *)value.c_str(), sizeof(DATA_REQUEST));
        
        Serial.printf("[Server] Max chunk size: %d \n", CHUNK_SIZE);

        Serial.printf("[Data request] Received request: Command %d, Alert TS: %llu, chunk size: %d\n", 
                     request.command, request.alertTimeStamp, request.chunkSize);
        alertTimeStamp = request.alertTimeStamp;
        if (CHUNK_SIZE != request.chunkSize)
        {
            Serial.println("Chunk sizes don't match!");
        }
        if(request.command == 1) { // Start new transfer
            Serial.println("Starting new transfer!");
            currentChunk = 0;
            currentSendingBufferIdx = 0; // Postaviti bufferIdx na proslih 5 sekundi
            
            int64_t minDiff = INT64_MAX;
            int minIdx = -1;

            // Find matching buffer (implementation depends on your storage logic)
            for(int i=0; i<NUMBER_OF_STORE_BUFFERS; i++) 
            {
                if(xSemaphoreTake(storeBufferMutex[i], 10) == pdTRUE)
                {
                    if(abs(int(storeBuffers[i].startTimeStamp - request.alertTimeStamp)) < minDiff) 
                    {
                        minDiff = abs(int(storeBuffers[i].startTimeStamp - request.alertTimeStamp));
                        transferBufferIdx = i;
                    }
                    xSemaphoreGive(storeBufferMutex[i]);
                }
            }
            totalChunks = ceil(sizeof(sendBuffer) / CHUNK_SIZE);
            Serial.printf("[Data request] Buffer with the closest start time: %d. Total chunks %d required with max chunk size %d.\n", transferBufferIdx, totalChunks, CHUNK_SIZE);                

            Serial.printf("Locking the store mutex to transfer the data!\n");
            if(xSemaphoreTake(storeBufferMutex[transferBufferIdx], 50) == pdTRUE) // busy wait 50 ms
            {
                
                int64_t alertTimeDiff = request.alertTimeStamp - storeBuffers[transferBufferIdx].startTimeStamp; 
                int sampleIdxDiff;
                if (alertTimeDiff > 0)
                {
                    alertEventIdx = ceil(alertTimeDiff / (int)samplingIntervalUs); // Index podatka na kom se dogodio alert. Zaokruzi na vise a ne truc da se posalje i sam alertIdx
                    Serial.printf("Alert detected at %d idx, value:0x%2x\n", alertEventIdx, storeBuffers[transferBufferIdx].buffer[alertEventIdx]);
                    Serial.printf("2 values before and after alert:\n0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n", 
                        storeBuffers[transferBufferIdx].buffer[alertEventIdx - 2],storeBuffers[transferBufferIdx].buffer[alertEventIdx - 1],storeBuffers[transferBufferIdx].buffer[alertEventIdx],
                        storeBuffers[transferBufferIdx].buffer[(alertEventIdx + 1) % STORE_BUFFER_SIZE], storeBuffers[transferBufferIdx].buffer[(alertEventIdx + 2) % STORE_BUFFER_SIZE]); // Moduo je za memory overflow
                    sampleIdxDiff = alertEventIdx - SEND_BUFFER_SIZE + 1; // da dobijem idx
                    if (sampleIdxDiff >= 0)
                    {
                        if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE)
                        {
                            buffersToSend[currentSendingBufferIdx].timeStamp = storeBuffers[transferBufferIdx].startTimeStamp + sampleIdxDiff * (int)samplingIntervalUs;
                            memcpy(buffersToSend[currentSendingBufferIdx].dataBufferPtr, &storeBuffers[transferBufferIdx].buffer[sampleIdxDiff], SEND_BUFFER_SIZE);
                            xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                        }
                    }
                    else
                    {
                        Serial.println("[Server] Negative difference, using two buffers to send data.");
                        // pronadji prethodni bafer
                        int prevTranserBuffer = transferBufferIdx == 0 ? NUMBER_OF_STORE_BUFFERS - 1 : transferBufferIdx - 1;
                        int prevStartIdx;
                        int prevSize;
                        // uzmi lock nad njim
                        Serial.printf("[Server] Taking lock of store buffer %d.\n", prevTranserBuffer);
                        if(xSemaphoreTake(storeBufferMutex[prevTranserBuffer], 20) == pdTRUE)
                        {
                            prevStartIdx = STORE_BUFFER_SIZE + sampleIdxDiff; // Odgovarajuca dimenzija
                            prevSize = abs(sampleIdxDiff); // koliko prethodnih elemenata
                            if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE)
                            {
                                buffersToSend[currentSendingBufferIdx].timeStamp = storeBuffers[transferBufferIdx].startTimeStamp - prevSize * (int)samplingIntervalUs; // Moramo oduzeti vrijeme
                                memcpy(buffersToSend[currentSendingBufferIdx].dataBufferPtr, &storeBuffers[prevTranserBuffer].buffer[prevStartIdx], prevSize);
                                xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                            }
                            else
                            {
                                Serial.printf("Unable to take a semaphore of send buffer to copy from previous buffer %d\n.", currentSendingBufferIdx);
                            }
                            xSemaphoreGive(storeBufferMutex[prevTranserBuffer]);
                        }
                        else
                        {
                            Serial.printf("[Server] Unsuccessfull lock of store buffer %d.\n", prevTranserBuffer);
                        }
                        if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE)
                        {
                            memcpy(&buffersToSend[currentSendingBufferIdx].dataBufferPtr[prevSize], &storeBuffers[transferBufferIdx].buffer[0], alertEventIdx + 1);
                            xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                        }
                        else
                        {
                            Serial.printf("Unable to take a semaphore of send buffer to copy from current buffer %d\n.",currentSendingBufferIdx);
                        }
                    }
                }
                Serial.printf("Releaseing the mutex. Data transfer complete!\n");
                xSemaphoreGive(storeBufferMutex[transferBufferIdx]);
                transferInProgress = true;
                sendChunk();
            }
            else
            {
                Serial.println("Failed to acquire store buffer mutex!");
            }
        }
        // Client ready for next chunk
        else if(request.command == 2) 
        {
            currentChunk++;
            sendChunk();
            Serial.printf("[Data request] Sending chunk %d / %d\n", currentChunk, totalChunks);
        }
        // End transfer of the past data
        else if(request.command == 3) 
        {
            Serial.println("[Data request] Past data transfer completed. Starting future timer!");
            currentSendingBufferIdx = 1;
            esp_timer_start_once(futureBufferTimer, 5.5 * 1000000); // Daj sebi 5.5 sekundi da se popuni 5s podataka. Smanjiti mozda.
        }
        // End transfer of all data
        else if(request.command == 4) 
        {
            currentSendingBufferIdx = 0;
            Serial.println("[Data request] Transfer completed.");
            transferInProgress = false;
            sendAlerts = true;
        }
    }
};

void sendChunk()
{
    if (DEBUG_PRINTS)
    {
        Serial.println("Sending chunks!");
    }
    Serial.printf("Locked the send mutex %d to send chunks!\n", currentSendingBufferIdx);
    if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], 10) == pdTRUE && transferInProgress) 
    {   
        // Everything can fit in a single message.
        uint16_t chunkSize;
        if (totalChunks == 0)
        {
            chunkSize = sizeof(sendBuffer);
        }
        else
        {
            chunkSize = (currentChunk == totalChunks-1) ? sizeof(sendBuffer) % CHUNK_SIZE : CHUNK_SIZE;
        }

        uint8_t chunk[chunkSize];
        uint8_t *currentBuffer = (uint8_t *)&buffersToSend[currentSendingBufferIdx];
        memcpy(chunk, &currentBuffer[currentChunk*CHUNK_SIZE], chunkSize);
        
        pTxData->setValue(chunk, chunkSize);
        pTxData->notify();
        
        Serial.printf("Sent %d bytes in chunk %d/%d\n", chunkSize, currentChunk, totalChunks);
        Serial.printf("Releasing the send mutex after chunks of buffer %d are sent!\n", currentSendingBufferIdx);
        xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
    } 
    else 
    {
        Serial.println("Failed to acquire send buffer lock!");
    }
}

// class ControlCallbacksLatency: public BLECharacteristicCallbacks {
//     void onWrite(BLECharacteristic *pChar) {
//         String value = pChar->getValue();
//         Serial.printf("Received value %s with length %d.\n", value, value.length());

//         if (startLatencyTest && value.length() == sizeof(uint64_t)) // value.length() == sizeof(uint64_t)
//         {
//             uint64_t clientSendTime = *((uint64_t*)value.c_str());
//             pLatencyControl->setValue((uint8_t*)&clientSendTime, sizeof(clientSendTime));
//             pLatencyControl->notify();
//             Serial.printf("Received [%d] timestamp: %llu\n", ++latencyCounter, clientSendTime);
//         }
//         else if (value == "PING") {
//             // Immediately echo back with server's receive time
//             latencyCounter = 0;
//             startLatencyTest = true;
//             uint64_t clientSendTime = 0;
//             pLatencyControl->setValue((uint8_t*)&clientSendTime, sizeof(clientSendTime));
//             pLatencyControl->notify();
//             Serial.println("Starting latency test.");
//         }
//         else if (value == "START_TIMESTAMPS") {
//             Serial.println("Switching to timestamp mode");
//             esp_timer_start_once(timestampTimer, 0);
//             delay(200);
//             do
//             {
//                 Serial.println("Initial timestamp is still not finished. Sleeping for 100 us.");
//                 delay(100);
//             } while(esp_timer_is_active(timestampTimer));
//             esp_timer_start_periodic(timestampTimer, 60 * 1000000); // Nece ga pokrenuti odmah, zato treba da odradimo prvo ovaj start_once() da se zabiljezi da je sve gotovo.
//             latencyTestMode = false;
//             startLatencyTest = false;
//         }
//     }
// };


void setup() {
    // setup serial for writing
    Serial.begin(115200);
    
    // Initialize LEDs
    for(int i = 0; i < NUMBER_OF_STORE_BUFFERS; i++) {
        pinMode(pinLayout[i], OUTPUT);
        digitalWrite(pinLayout[i], LOW);
    }
    digitalWrite(activePin, HIGH); // Active buffer LED on
    if (DEBUG_PRINTS)
    {
        Serial.println("Buffer pins initialized.");
    }
    
    // Initialize buffers
    memset(storeBuffers, 0, sizeof(storeBuffers)); // buffers for storing samples
    // memset(&transferBuffer, 0, sizeof(transferBuffer)); // buffer for sending out time window
    currentStoreBufferIdx = 0;
    currentSendingBufferIdx = 0;
    if (DEBUG_PRINTS)
    {
        Serial.println("Reset all buffers.");
    }

    for(int i = 0; i < NUMBER_OF_STORE_BUFFERS; i++) 
    {
        storeBufferMutex[i] = xSemaphoreCreateMutex();
        if(storeBufferMutex[i] == NULL) {
            Serial.println("Store mutex creation failed!");
            while(1); // Critical failure
        }
    }
    if (DEBUG_PRINTS)
    {
        Serial.println("Store mutex creation finished.");
    }

    for(int i = 0; i < NUMBER_OF_SEND_BUFFERS; i++) {
        // kreiraj mutekse
        sendBufferMutex[i] = xSemaphoreCreateMutex();
        if(sendBufferMutex[i] == NULL) {
            Serial.println("Send mutex creation failed!");
            while(1); // Critical failure
        }
    }
    if (DEBUG_PRINTS)
    {
        Serial.println("Send mutex creation finished.");
    }

    // Initialize BLE
    BLEDevice::init("ESP32_Server_Alert_Data_Transmission");
    BLEDevice::setMTU(START_MTU_SIZE); // Request larger MTU
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
    if (DEBUG_PRINTS)
    {
        Serial.println("Transmission data characteristic initialized.");
    }
    delay(100);
    
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
    if (DEBUG_PRINTS)
    {
        Serial.println("Time-stamp characteristic initialized.");
    }
    delay(100);

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
    if (DEBUG_PRINTS)
    {
        Serial.println("Alert data characteristic initialized.");
    }
    delay(100);

    // // Latency Control Characteristic (write and notify)
    // pLatencyControl = pService->createCharacteristic(
    //     LATENCY_UUID,
    //     BLECharacteristic::PROPERTY_WRITE | 
    //     BLECharacteristic::PROPERTY_NOTIFY
    // );
    // pLatencyControl->setCallbacks(new ControlCallbacksLatency());
    // pLatencyControl->addDescriptor(new BLE2902());
    // // Latency descriptor
    // BLE2901* pLatencyDesc = new BLE2901();
    // pLatencyDesc->setDescription("Latency control.");
    // pLatencyControl->addDescriptor(pLatencyDesc);
    // // delay after initializing characteristic
    // if (DEBUG_PRINTS)
    // {
    //     Serial.println("Latency characteristic initialized.");
    // }
    
    // RX Control Characteristic (Write)
    pRxControl = pService->createCharacteristic(
        RX_CONTROL_UUID,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY 
    );
    pRxControl->setCallbacks(new RxControlCallback());
    // Add description
    BLE2901* pRxDesc = new BLE2901();
    pRxDesc->setDescription("Data transfer controll callback.");
    pRxControl->addDescriptor(pRxDesc);
    // delay after initializing characteristic
    if (DEBUG_PRINTS)
    {
        Serial.println("Control characteristic initialized.");
    }

    
    // // Latency Control Characteristic (write and notify)
    // pLatencyControl = pService->createCharacteristic(
    //     LATENCY_UUID,
    //     BLECharacteristic::PROPERTY_WRITE | 
    //     BLECharacteristic::PROPERTY_NOTIFY
    // );
    // pLatencyControl->setCallbacks(new ControlCallbacksLatency());
    // pLatencyControl->addDescriptor(new BLE2902());
    // // Latency descriptor
    // BLE2901* pLatencyDesc = new BLE2901();
    // pLatencyDesc->setDescription("Latency control.");
    // pLatencyControl->addDescriptor(pLatencyDesc);
    // // delay after initializing characteristic
    // if (DEBUG_PRINTS)
    // {
    //     Serial.println("Latency characteristic initialized.");
    // }
    
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
    esp_log_level_set("BLE", ESP_LOG_VERBOSE);
    esp_log_level_set("GATT", ESP_LOG_VERBOSE);

    // BLE MAC
    String bleMac = BLEDevice::getAddress().toString();
    Serial.println("BLE MAC Address: " + bleMac);

    // After creating all characteristics
    Serial.println("Server characteristics created:");
    if (DEBUG_PRINTS)
    {
        Serial.printf("- TX Data: %s\n", pTxData->getUUID().toString().c_str());
        Serial.printf("- RX Control: %s\n", pRxControl->getUUID().toString().c_str());
        Serial.printf("- Timestamp: %s\n", pTimestampChar->getUUID().toString().c_str());
        // Serial.printf("- Latency: %s\n", pLatencyControl->getUUID().toString().c_str());
        Serial.printf("- Alert: %s\n", pAlertMeasurementChar->getUUID().toString().c_str());
    }

    pService->start();

    Serial.printf(" TX Data Characteristic handle: %d\n", pTxData->getHandle());
    Serial.printf(" Timestamp Characteristic handle: %d\n", pTimestampChar->getHandle());
    Serial.printf(" Alert Characteristic handle: %d\n", pAlertMeasurementChar->getHandle());
    // Serial.printf(" Latency Characteristic handle: %d\n", pLatencyControl->getHandle());
    Serial.printf(" RX Control Characteristic handle: %d\n", pRxControl->getHandle());
    

    // Configure advertising properly
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Helps with iPhone connections
    BLEDevice::startAdvertising();
    advertising = true;
    if (DEBUG_PRINTS)
    {
        Serial.println("Started advertising.");
    }
    Serial.println("Server ready for connections");
    
    // Setup sampling timer
    esp_timer_create_args_t sampleTimerArgs = {
        .callback = &sampleData,
        .name = "dataSampler"
    };
    esp_timer_create(&sampleTimerArgs, &sampleTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("Created sample timer.");
    }
    
    // Setup timestamp timer
    esp_timer_create_args_t tsTimerArgs = {
        .callback = &sendTimestamp,
        .name = "timestampNotifier"
    };
    esp_timer_create(&tsTimerArgs, &timestampTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("Created time-stamp timer.");
    }

    // Setup future buffer timer
    esp_timer_create_args_t futureBufferTimerArgs = {
        .callback = &fillFutureBuffer,
        .name = "futureBuffer"
    };
    esp_timer_create(&futureBufferTimerArgs, &futureBufferTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("Created future buffer timer.");
    }
    
    // Start timers
    esp_timer_start_periodic(sampleTimer, samplingIntervalUs); // TODO: This should be changed?
    Serial.println("Started sampling timer with sampling interval:");
    Serial.println(samplingIntervalUs);
    // esp_timer_start_periodic(timestampTimer, 60 * 1000000); // 60 seconds
    
    Serial.println("BLE alert data Ready");
}

void loop() {
    // Start advertising if the device is disconnected and advertisiment stopped
    if (!deviceConnected && !advertising) {
        BLEDevice::startAdvertising();
        advertising = true;
        Serial.println("Restarting advertising");
    }
    delay(50); // Prevent watchdog reset
}