#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "../power_signal_data_long.h"
#include "../communication_structures.h"
#include "../constants.h"

// store buffer variables
STORE_BUFFER storeBuffers[NUMBER_OF_STORE_BUFFERS];
sendBuffer buffersToSend[NUMBER_OF_SEND_BUFFERS];

// mutex init
SemaphoreHandle_t storeBufferMutex[NUMBER_OF_STORE_BUFFERS]; // One mutex per store buffer
SemaphoreHandle_t sendBufferMutex[NUMBER_OF_SEND_BUFFERS]; // One mutex per send buffer
// SemaphoreHandle_t alertMutex;

// Debug handling stuff
bool DEBUG_PRINTS = true;

uint8_t currentStoreBufferIdx = 0;
uint8_t currentSendingBufferIdx = 0;

// BLE Objects
BLEServer *pServer;
BLECharacteristic *pTxData;
BLECharacteristic *pRxControl;
BLECharacteristic *pTimestampChar;
// BLECharacteristic *pLatencyControl; // For latency characteristic
BLECharacteristic *pAlertMeasurementChar; // For incorrect measurement


volatile bool deviceConnected = false;
volatile bool advertising = false;
// bool latencyTestMode = true;
// bool startLatencyTest = false; // Vrijednost koja pokazuje da li pocinjemo ili ne sa mjerenjem vremena, podesava se na true sa prvim pingom
volatile bool sendAlerts = false; // Vrijednost koja pokazuje da li pocinjemo ili ne sa mjerenjem vremena, podesava se na true sa prvim pingom
volatile bool untilTimestamp = true;

uint32_t sampleCounter = 0;
uint32_t currentSampleCounter = 0;
uint32_t bufferCounters[NUMBER_OF_STORE_BUFFERS] = {0};
uint64_t lastSampleTime = 0;

// Globalna promjenljiva za alert indeks
// TODO: Dodati nekakav semafor za ovo.
volatile bool alertDetected = false;
int alertEventIdx;
uint64_t alertTimeStamp; // Za future.
int16_t currentThresholdValue = DEFAULT_THRESHOLD_CURRENT;
int16_t voltageThresholdValue = DEFAULT_THRESHOLD_VOLTAGE;

// Buffer transfer control
bool waitForAck = false;
int alertStoreBufferIdx = 0;

// data transfer values
volatile bool transferInProgress = false;
uint16_t currentChunk = 0;
uint16_t totalChunks = 0;
// int sendBufferIdx;


uint16_t CHUNK_SIZE = DEFAULT_CHUNK_SIZE; // Default chunk size. Assign new value according to the MTU size (MTU - 3)

// Timer handles
float samplingIntervalUs = (1.f / SAMPLING_FREQUENCY) * 1'000'000;

esp_timer_handle_t sampleTimer;
esp_timer_handle_t timestampTimer;
esp_timer_handle_t futureBufferTimer;

// future buffer callback
void fillFutureBuffer(void* arg)
{
    if(xSemaphoreTake(storeBufferMutex[alertStoreBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE) // busy wait 5 ms
    {
        if (DEBUG_PRINTS)
        {
            Serial.printf("[Future buffer] Alert detected at %d idx, current value: %d, voltage value: %d\n", 
                alertEventIdx, storeBuffers[alertStoreBufferIdx].currentBuffer[alertEventIdx], storeBuffers[alertStoreBufferIdx].voltageBuffer[alertEventIdx]);
        }
        int endBufferIdx = alertEventIdx + SEND_BUFFER_ELEMENTS; // AlertEventIdx se ovdje svakako vec zna
        // Ako se ovo dogodi, to znaci da nam treba jos jedan buffer
        if (endBufferIdx > STORE_BUFFER_ELEMENTS)
        {
            // Razlika izmedju endBufferIdx i STORE_BUFFER_SIZE ce nam dati koliko treba narednog da se uzme a trenutnog je SEND_BUFFER_SIZE - razlika
            int overheadIndices = endBufferIdx - STORE_BUFFER_ELEMENTS; // 153 - 150 = 3; Znaci treba nam do idx 150 (ne racunajuci njega) onda nam treba 150, 151, 152 (3 komada)
            int firstBufferSize = SEND_BUFFER_ELEMENTS - overheadIndices;            
            // Calculate next transfer buffer idx
            int nextAlertStoreBufferIdx = alertStoreBufferIdx == (NUMBER_OF_STORE_BUFFERS - 1) ? 0 : alertStoreBufferIdx + 1; // Ako je transferBuffer 2, tada je iduci 0
            if (DEBUG_PRINTS)
            {
                Serial.println("[Future buffer] Data after alert is in two buffers.");
                Serial.printf("[Future buffer] Copying first batch of %d indices.\n", firstBufferSize);
            }
            if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
            {
                buffersToSend[currentSendingBufferIdx].timeStamp = alertTimeStamp;
                memcpy(buffersToSend[currentSendingBufferIdx].currentBuffer, &storeBuffers[alertStoreBufferIdx].currentBuffer[alertEventIdx], sizeof(uint16_t) * firstBufferSize);
                memcpy(buffersToSend[currentSendingBufferIdx].voltageBuffer, &storeBuffers[alertStoreBufferIdx].voltageBuffer[alertEventIdx], sizeof(uint16_t) * firstBufferSize);
                xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
            }
            else
            {
                Serial.printf("[Future buffer] Failed to fetch send buffer %d mutex\n.", currentSendingBufferIdx);
            }

            
            if(xSemaphoreTake(storeBufferMutex[nextAlertStoreBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE) // busy wait 50 ms
            {
                if (DEBUG_PRINTS)
                {
                    Serial.printf("[Future buffer] Copying second batch of %d indices.\n", overheadIndices);
                }
                if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
                {
                    memcpy(buffersToSend[currentSendingBufferIdx].currentBuffer, &storeBuffers[nextAlertStoreBufferIdx].currentBuffer[0], sizeof(uint16_t) * overheadIndices);
                    memcpy(buffersToSend[currentSendingBufferIdx].voltageBuffer, &storeBuffers[nextAlertStoreBufferIdx].voltageBuffer[0], sizeof(uint16_t) * overheadIndices);
                    xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                }
                else
                {
                    Serial.println("[Future buffer] Failed to fetch send buffer mutex.");
                }    
                xSemaphoreGive(storeBufferMutex[nextAlertStoreBufferIdx]); // Release current
            }
            else
            {
                Serial.printf("[Future buffer] Problem with allocating store buffer %d", nextAlertStoreBufferIdx);
            }        
        }
        else
        {
            if (DEBUG_PRINTS)
            {
                Serial.println("[Future buffer] Data after alert is in one buffer.");
            }
            if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
            {
                buffersToSend[currentSendingBufferIdx].timeStamp = alertTimeStamp;
                memcpy(buffersToSend[currentSendingBufferIdx].currentBuffer, &storeBuffers[alertStoreBufferIdx].currentBuffer[alertEventIdx], sizeof(uint16_t) * SEND_BUFFER_SIZE);
                memcpy(buffersToSend[currentSendingBufferIdx].voltageBuffer, &storeBuffers[alertStoreBufferIdx].voltageBuffer[alertEventIdx], sizeof(uint16_t) * SEND_BUFFER_SIZE);
                xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
            }
            else
            {
                Serial.printf("[Future buffer] Failed to fetch send buffer mutex %d.\n", currentSendingBufferIdx);
            }
        }
		Serial.println("[Future buffer] Future buffer filled.");
        xSemaphoreGive(storeBufferMutex[alertStoreBufferIdx]); // Release current
        //printHexValues();
        sendChunk();
    }
    else
    {
        Serial.printf("[Failed Future buffer] Problem with allocating store buffer %d", alertStoreBufferIdx);
    }        
}


// Sampling callback
void sampleData(void* arg) {
    // declare variable where data should be stored at
    uint64_t now = esp_timer_get_time();
    int16_t electricCurrentData;
    int16_t voltageData;
    STORE_BUFFER *currentBuffer;
    // Try to lock current buffer (non-blocking)
    if(xSemaphoreTake(storeBufferMutex[currentStoreBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)  // Minimalno blocking vrijeme mora biti manje od (1/SAMPLING_FREQUENCY)/2 10 ms / 2 je 5
    {
        // Get current buffer
        currentBuffer = &storeBuffers[currentStoreBufferIdx];

        // Podijeli sa modulom STORE_BUFFER_SIZE kad se uzima pun bafer pa da ide ispocetka
        currentBuffer->sampleIdx %= STORE_BUFFER_ELEMENTS;

        // Initialize timestamp on first sample
        if (currentBuffer->sampleIdx == 0) {
            currentBuffer->startTimeStamp = now;
        }
        
        // Store data (circular buffer within LOGGED_DATA)
        electricCurrentData = CURRENT_SIGNAL[sampleCounter % TOTAL_SAMPLES]; // This can be read from ADC
        voltageData = VOLTAGE_SIGNAL[sampleCounter % TOTAL_SAMPLES]; // This can be read from ADC
        currentBuffer->currentBuffer[currentBuffer->sampleIdx] = electricCurrentData;
        currentBuffer->voltageBuffer[currentBuffer->sampleIdx] = voltageData;

        // Alerts cannot be triggered if current alert is being processed
        if (sendAlerts && (abs(electricCurrentData) >= currentThresholdValue || abs(voltageData) >= voltageThresholdValue)) // lazy evaluation, also add abs to cover positive and negative values! 
        {
            alertDetected = true;
            alertTimeStamp = now;
            alertEventIdx = currentBuffer->sampleIdx;
            if (DEBUG_PRINTS)
            {
                Serial.println("[Alert sync] Alert the client that server discovered abnormality.");
                Serial.printf("[Alert sync] Time of detection: %llu - idx: %d, current threshold: %d [A], voltage threshold: %d [V].\n[Alert sync] Measured value: %d [A] %d [V]\n", now, alertEventIdx, currentThresholdValue, voltageThresholdValue, electricCurrentData, voltageData);
            }
        }

        sampleCounter++;
        currentBuffer->sampleIdx++;

        
        // Switch buffers when current one is full
        if (currentBuffer->sampleIdx >= STORE_BUFFER_ELEMENTS) 
        {
            uint8_t originalIdx = currentStoreBufferIdx;
            currentStoreBufferIdx = (currentStoreBufferIdx + 1) % NUMBER_OF_STORE_BUFFERS;
            xSemaphoreGive(storeBufferMutex[originalIdx]);
        }
        else
        {
            xSemaphoreGive(storeBufferMutex[currentStoreBufferIdx]);
        }
        
        // Trigger alert characteristic
        if (deviceConnected && alertDetected && sendAlerts) // TODO: Mozda bih mogao samo sendAlerts onemoguciti dok se ne zavrsi proces transferInProgress
        {
            if (DEBUG_PRINTS)
            {
                Serial.println("[Alert sync] Sending notify for alert. Disable other Alerts.");
            }
            sendAlerts = false;
            pAlertMeasurementChar->setValue((uint8_t *)&alertTimeStamp, sizeof(uint64_t));
            pAlertMeasurementChar->notify();
        }
    }
    else 
    {
        Serial.printf("[Failed Dropping sample] Buffer %d locked, dropping the sample.\n", currentStoreBufferIdx);
    }
}

// Timestamp notification callback
void sendTimestamp(void* arg) {
    if (deviceConnected) {
        uint64_t now = esp_timer_get_time();
        pTimestampChar->setValue((uint8_t *)&now, 8);
        pTimestampChar->notify();
        // TODO: Ovdje dodati nekakav if ako je transfer u toku da se ne dira
        if (untilTimestamp)
        {
            // xSemaphoreGive(alertMutex);
            Serial.println("[Time stamp] Enabling the alert sending.");
            sendAlerts = true;
            untilTimestamp = false;
        }
        if (DEBUG_PRINTS)
        {
            Serial.printf("[Time stamp] Time sent: %llu µs\n", now);
        }
    }
}

// BLE Server Callbacks
class ServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
        deviceConnected = true;
        Serial.println("[Server on connect] Client connected");
        CHUNK_SIZE =  pServer->getPeerMTU(pServer->getConnId()) - 3;
        if (DEBUG_PRINTS)
        {
            Serial.printf("[Server on connect] MTU size: %d\n", pServer->getPeerMTU(pServer->getConnId()));
            Serial.printf("[Server on connect] Max chunk size: %d B\n", CHUNK_SIZE);
            Serial.printf("[Server on connect] Store buffer elements: %d store buffer size: %d.\n", STORE_BUFFER_ELEMENTS, STORE_BUFFER_SIZE);
            Serial.println("[Server on connect] Disabling alerts.");
        }
        // Restart timstamp notifications on reconnect
        sendAlerts = false;

        esp_timer_start_periodic(timestampTimer, TIMESYNC_INTERVAL_US);
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        sendAlerts = false;
        if (DEBUG_PRINTS)
        {
            Serial.println("[On disconnect] Disabeling alerts.");
        }
        CHUNK_SIZE = DEFAULT_CHUNK_SIZE;
        Serial.println("[On disconnect] Client disconnected");
        esp_timer_stop(timestampTimer);
        currentThresholdValue = DEFAULT_THRESHOLD_VOLTAGE;
        voltageThresholdValue = DEFAULT_THRESHOLD_CURRENT;
        // Oslobodi sve semafore 
        for (int i = 0; i < NUMBER_OF_STORE_BUFFERS; i++)
        {
            xSemaphoreGive(storeBufferMutex[i]);
        }
        for (int i = 0; i < NUMBER_OF_SEND_BUFFERS; i++)
        {
            xSemaphoreGive(sendBufferMutex[i]);
        }

        // Critical fixes:
        BLEDevice::startAdvertising();
        advertising = true;
        delay(500); // Allow advertising to start
    }

    void onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* desc) 
    {
        CHUNK_SIZE =  pServer->getPeerMTU(pServer->getConnId()) - 3;
        Serial.printf("[Server] New MTU: %d\nMax chunk size: %d\n", CHUNK_SIZE + 3, CHUNK_SIZE);
    }
};

class RxControlCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
        Serial.println("[Data request] Received message! Disabeling alerts.");
        sendAlerts = false;
        String value = pCharacteristic->getValue();
        DATA_REQUEST request;
        // cast const char* to DATA_REQUEST struct
        memcpy(&request, (uint8_t *)value.c_str(), sizeof(DATA_REQUEST));
        if (DEBUG_PRINTS)
        {
            Serial.printf("[Data request] Max chunk size: %d \n", CHUNK_SIZE);
            Serial.printf("[Data request] Received request: Command %d, Alert TS: %llu\n", 
                request.command, request.alertTimeStamp);
        }
        // If alert is on this device, there is no need to read data.
        if (!alertDetected)
        {
            alertTimeStamp = request.alertTimeStamp;
        }     
        // Start new transfer
        if(request.command == 1) 
        { 
            Serial.println("[Data request] Starting new transfer!");
            currentChunk = 0;
            // set sending buffer idx to past buffer
            currentSendingBufferIdx = 0; 
            
            int64_t minDiff = INT64_MAX;
            alertStoreBufferIdx = -1;
            // Find matching buffer (implementation depends on your storage logic)
            for(int i=0; i<NUMBER_OF_STORE_BUFFERS; i++) 
            {
                if(xSemaphoreTake(storeBufferMutex[i], pdMS_TO_TICKS(1)) == pdTRUE) // busy wait  5 ms
                {
                    if(abs(int(storeBuffers[i].startTimeStamp - alertTimeStamp)) < minDiff) 
                    {
                        minDiff = abs(int(storeBuffers[i].startTimeStamp - alertTimeStamp));
                        alertStoreBufferIdx = i;
                    }
                    xSemaphoreGive(storeBufferMutex[i]);
                }
                else
                {
                    Serial.printf("[Failed Data request] Unable to lock store buffer %d.\n", i);
                }
            }
            // TODO: Sta ako ne nadjem alertStoreBufferIdx, mozda vrtiti u while petlji?
            if (alertStoreBufferIdx == -1)
            {
                Serial.println("[Data request] Nema bafera, treba vrtiti.");
            }
            totalChunks = (int)ceil(sizeof(sendBuffer) / (float)CHUNK_SIZE);
            if (DEBUG_PRINTS)
            {
                Serial.printf("[Data request] Buffer with the closest start time: %d. Total chunks %d required with max chunk size %d.\n", alertStoreBufferIdx, totalChunks, CHUNK_SIZE);                
                Serial.printf("[Data request] Locking the store mutex to transfer the data!\n");
            }
            // Copy data.
            if(xSemaphoreTake(storeBufferMutex[alertStoreBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE) // busy wait 5 ms
            {
                // Find alertIdx
                int sampleIdxDiff; 
                if (!alertDetected)
                {
                    int64_t alertTimeDiff = alertTimeStamp - storeBuffers[alertStoreBufferIdx].startTimeStamp; 
                    if (alertTimeDiff < 0)
                    {
                        Serial.println("[Data request] Alert time diff is negative! Taking previous buffer");
                        // Ovdje sada treba da preuzmemo prethodni bafer index.
                        int prevAlertBufferIdx = (alertStoreBufferIdx == 0) ? NUMBER_OF_STORE_BUFFERS - 1 : alertStoreBufferIdx - 1;
                        if(xSemaphoreTake(storeBufferMutex[prevAlertBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
                        {
                            // Give old buffer
                            xSemaphoreGive(storeBufferMutex[alertStoreBufferIdx]);
                            alertStoreBufferIdx = prevAlertBufferIdx;
                            alertTimeDiff = alertTimeStamp - storeBuffers[alertStoreBufferIdx].startTimeStamp; 

                        }
                        else
                        {
                            Serial.printf("[Failed Data request] Unable to acquire previous store buffer: %d.\n", prevAlertBufferIdx);
                        }
                    }
                    alertEventIdx = alertTimeDiff / SAMPLE_INTERVAL_US; // (int)samplingIntervalUs Index podatka na kom se dogodio alert. Zaokruzi na vise a ne truc da se posalje i sam alertIdx
                    if (DEBUG_PRINTS)
                    {
                        Serial.printf("[Alert on timer] Alert detected at %d idx, current value: %d, voltage value: %d\n", 
                            alertEventIdx, storeBuffers[alertStoreBufferIdx].currentBuffer[alertEventIdx], 
                            storeBuffers[alertStoreBufferIdx].voltageBuffer[alertEventIdx]
                        );
                    }
                }
                sampleIdxDiff = alertEventIdx - SEND_BUFFER_ELEMENTS + 1; // Dobijamo pocetni idx
                if (sampleIdxDiff >= 0)
                {
                    if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
                    {
                        buffersToSend[currentSendingBufferIdx].timeStamp = storeBuffers[alertStoreBufferIdx].startTimeStamp + sampleIdxDiff * SAMPLE_INTERVAL_US; // (int)samplingIntervalUs
                        memcpy(buffersToSend[currentSendingBufferIdx].currentBuffer, &storeBuffers[alertStoreBufferIdx].currentBuffer[sampleIdxDiff], sizeof(uint16_t) * SEND_BUFFER_SIZE);
                        memcpy(buffersToSend[currentSendingBufferIdx].voltageBuffer, &storeBuffers[alertStoreBufferIdx].voltageBuffer[sampleIdxDiff], sizeof(uint16_t) * SEND_BUFFER_SIZE);
                        xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                    }
                    else
                    {
                        Serial.printf("[Failed Data request] Command 1, unable to lock send buffer %d.\n", currentSendingBufferIdx);
                    }
                }
                else
                {
                    int prevAlertStoreBufferIdx = alertStoreBufferIdx == 0 ? NUMBER_OF_STORE_BUFFERS - 1 : alertStoreBufferIdx - 1; // prethodni bafer
                    int prevStartIdx = STORE_BUFFER_ELEMENTS + sampleIdxDiff; // Odgovarajuca dimenzija
                    int prevSize = abs(sampleIdxDiff); // koliko prethodnih elemenata
                    if (DEBUG_PRINTS)
                    {
                        Serial.println("[Data request] Negative difference, using two buffers to send data.");
                        Serial.printf("[Data request] Taking lock of store buffer %d.\n", prevAlertStoreBufferIdx);
                    }
                    // Popuni prvo iz proslog store bafera
                    if(xSemaphoreTake(storeBufferMutex[prevAlertStoreBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
                    {
                        if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
                        {
                            buffersToSend[currentSendingBufferIdx].timeStamp = storeBuffers[alertStoreBufferIdx].startTimeStamp - prevSize * SAMPLE_INTERVAL_US; // Moramo oduzeti vrijeme SAMPLE_INTERVAL_US
                            memcpy(buffersToSend[currentSendingBufferIdx].currentBuffer, &storeBuffers[prevAlertStoreBufferIdx].currentBuffer[prevStartIdx], sizeof(uint16_t) * prevSize);
                            memcpy(buffersToSend[currentSendingBufferIdx].voltageBuffer, &storeBuffers[prevAlertStoreBufferIdx].voltageBuffer[prevStartIdx], sizeof(uint16_t) * prevSize);
                            xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                        }
                        else
                        {
                            Serial.printf("[Failed Data request] Command 1, unable to lock send buffer %d.\n", currentSendingBufferIdx);
                        }
                        xSemaphoreGive(storeBufferMutex[prevAlertStoreBufferIdx]);
                    }
                    else
                    {
                        Serial.printf("[Failed Data request] Command 1, unable lock of store buffer %d.\n", prevAlertStoreBufferIdx);
                    }
                    // Popuni iz trenutnog store bafera
                    if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE)
                    {
                        memcpy(&buffersToSend[currentSendingBufferIdx].currentBuffer[prevSize], &storeBuffers[alertStoreBufferIdx].currentBuffer[0], sizeof(uint16_t) * (alertEventIdx + 1));
                        memcpy(&buffersToSend[currentSendingBufferIdx].voltageBuffer[prevSize], &storeBuffers[alertStoreBufferIdx].voltageBuffer[0], sizeof(uint16_t) * (alertEventIdx + 1));
                        xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
                    }
                    else
                    {
                        Serial.printf("[Failed Data request] Command 1, unable to lock send buffer %d\n.", currentSendingBufferIdx);
                    }
                }
                if (DEBUG_PRINTS)
                {
                    Serial.printf("[Data request] Releaseing the mutex. Data transfer complete!\n");
                }              
                xSemaphoreGive(storeBufferMutex[alertStoreBufferIdx]);
                transferInProgress = true;
                sendChunk();
            }
            else
            {
                Serial.printf("[Data request] Command 1, failed to acquire store buffer %d mutex!\n", alertStoreBufferIdx);
            }
        }
        // Client ready for next chunk
        else if(request.command == 2) 
        {
            currentChunk++;
            sendChunk();
            if (DEBUG_PRINTS)
            {
                Serial.printf("[Data request] Sending chunk %d / %d\n", currentChunk, totalChunks);
            }
        }
        // End transfer of the past data
        else if(request.command == 3) 
        {
            currentChunk = 0;
            Serial.println("[Data request] Past data transfer completed. Starting future timer!");
            currentSendingBufferIdx = 1;
            esp_timer_start_once(futureBufferTimer, 0.11 * 1'000'000); // Daj sebi 5.5 sekundi da se popuni 5s podataka. Smanjiti mozda.
        }
        // End transfer of all data
        else if(request.command == 4) 
        {
            currentSendingBufferIdx = 0;
            Serial.println("[Data request] Transfer completed. Allowing alerts to be sent again.");
            transferInProgress = false;
            if (request.reconstructionTriggered)
            {
                currentThresholdValue = request.currentThresholdValue;
                voltageThresholdValue = request.voltageThresholdValue;
                Serial.printf("[Threshold change] New threshold values: %d [V] %d [A].\n", voltageThresholdValue, currentThresholdValue);
            }
            sendAlerts = true;
            alertDetected = false;
        }
    }
};


void sendChunk()
{
    if (DEBUG_PRINTS)
    {
        Serial.printf("[Send chunks] Locking the send mutex %d to send chunks!\n", currentSendingBufferIdx);
    }
    if(xSemaphoreTake(sendBufferMutex[currentSendingBufferIdx], pdMS_TO_TICKS(1)) == pdTRUE && transferInProgress) 
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
        if (DEBUG_PRINTS)
        {
            Serial.printf("[Send chunks] Sent %d bytes in chunk %d/%d\n", chunkSize, currentChunk, totalChunks);
            Serial.printf("[Send chunks] Releasing the send mutex after chunks of buffer %d are sent!\n", currentSendingBufferIdx);
        }
        xSemaphoreGive(sendBufferMutex[currentSendingBufferIdx]);
    } 
    else 
    {
        Serial.printf("[Failed Send chunks] Failed to acquire send buffer %d lock!\n", currentSendingBufferIdx);
    }
}


void setup() {
    // setup serial for writing
    Serial.begin(115200);
    
    // Initialize buffers
    memset(storeBuffers, 0, sizeof(storeBuffers)); // buffers for storing samples
    memset(buffersToSend, 0, sizeof(buffersToSend)); // buffers for storing samples
    currentStoreBufferIdx = 0;
    currentSendingBufferIdx = 0;
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Reset all buffers.");
    }

    for(int i = 0; i < NUMBER_OF_STORE_BUFFERS; i++) 
    {
        storeBufferMutex[i] = xSemaphoreCreateMutex();
        if(storeBufferMutex[i] == NULL) {
            Serial.println("[Setup] Store mutex creation failed!");
            while(1); // Critical failure
        }
    }
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Store mutex creation finished.");
    }

    for(int i = 0; i < NUMBER_OF_SEND_BUFFERS; i++) {
        // kreiraj mutekse
        sendBufferMutex[i] = xSemaphoreCreateMutex();
        if(sendBufferMutex[i] == NULL) {
            Serial.println("[Setup] Send mutex creation failed!");
            while(1); // Critical failure
        }
    }
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Send mutex creation finished.");
    }

    // Initialize BLE
    BLEDevice::init("ESP32_Server_Data_Transmission_C");
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
    pTxDesc->setDescription("Measured values.");
    pTxData->addDescriptor(pTxDesc);
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Transmission data characteristic initialized.");
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
        Serial.println("[Setup] Time-stamp characteristic initialized.");
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
    pAlertDesc->setDescription("Alert notify characteristic for data transfer.");
    pAlertMeasurementChar->addDescriptor(pAlertDesc);
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Alert data characteristic initialized.");
    }
    delay(100);

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
        Serial.println("[Setup] Control characteristic initialized.");
    }

    // Alert threshold value:
    currentThresholdValue = DEFAULT_THRESHOLD_CURRENT;
    voltageThresholdValue = DEFAULT_THRESHOLD_VOLTAGE;

    // BLE MAC
    String bleMac = BLEDevice::getAddress().toString();
    Serial.println("[Setup] BLE MAC Address: " + bleMac);

    // After creating all characteristics
    Serial.println("[Setup] Server characteristics created:");
    if (DEBUG_PRINTS)
    {
        Serial.printf("- TX Data: %s\n", pTxData->getUUID().toString().c_str());
        Serial.printf("- RX Control: %s\n", pRxControl->getUUID().toString().c_str());
        Serial.printf("- Timestamp: %s\n", pTimestampChar->getUUID().toString().c_str());
        // Serial.printf("- Latency: %s\n", pLatencyControl->getUUID().toString().c_str());
        Serial.printf("- Alert: %s\n", pAlertMeasurementChar->getUUID().toString().c_str());
    }

    pService->start();
    if (DEBUG_PRINTS)
    {
        Serial.printf("- TX Data Characteristic handle: %d\n", pTxData->getHandle());
        Serial.printf("- Timestamp Characteristic handle: %d\n", pTimestampChar->getHandle());
        Serial.printf("- Alert Characteristic handle: %d\n", pAlertMeasurementChar->getHandle());
        // Serial.printf(" Latency Characteristic handle: %d\n", pLatencyControl->getHandle());
        Serial.printf("- RX Control Characteristic handle: %d\n", pRxControl->getHandle());
    }

    // Configure advertising properly
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Helps with iPhone connections
    BLEDevice::startAdvertising();
    advertising = true;
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Started advertising.");
    }
    Serial.println("[Setup] Server ready for connections");
    
    // Setup sampling timer
    esp_timer_create_args_t sampleTimerArgs = {
        .callback = &sampleData,
        .name = "dataSampler"
    };
    esp_timer_create(&sampleTimerArgs, &sampleTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Created sample timer.");
    }
    
    // Setup timestamp timer
    esp_timer_create_args_t tsTimerArgs = {
        .callback = &sendTimestamp,
        .name = "timestampNotifier"
    };
    esp_timer_create(&tsTimerArgs, &timestampTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Created time-stamp timer.");
    }

    // Setup future buffer timer
    esp_timer_create_args_t futureBufferTimerArgs = {
        .callback = &fillFutureBuffer,
        .name = "futureBuffer"
    };
    esp_timer_create(&futureBufferTimerArgs, &futureBufferTimer);
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Created future buffer timer.");
    }
    
    // Start timers
    esp_timer_start_periodic(sampleTimer, SAMPLE_INTERVAL_US); // TODO: This should be changed?
    Serial.printf("[Setup] Started sampling timer with sampling interval: %d - %f\n", SAMPLE_INTERVAL_US, samplingIntervalUs);
    
    Serial.println("[Setup] Server C - BLE alert data Ready");
}

void loop() {
    // Start advertising if the device is disconnected and advertisiment stopped
    if (!deviceConnected && !advertising) {
        BLEDevice::startAdvertising();
        advertising = true;
        Serial.println("[Loop] Restarting advertising");
    }
    delay(50); // Prevent watchdog reset
}