#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "../constants.h"
#include "../communication_structures.h"
#include "../power_signal_data_long.h"

#define NUMBER_OF_SERVERS       3
#define NUMBER_OF_SEND_BUFFERS  2
#define QUEUE_SIZE              10
#define MAX_QUEUE_ITEM_SIZE     256

#define COMBINED_ELEMENTS       90
#define FIRST_BUFFER_SKIP       6
#define SECOND_BUFFER_SKIP      7
// combinedElements = 90, firstBufferSkip = 6, secondBufferSkip = 7.

// Server addresses
static BLEAddress serverAddress_A(ESP_A_MAC);
static BLEAddress serverAddress_C(ESP_C_MAC);
static BLEAddress serverAddress_D(ESP_D_MAC);

// Debug control
bool DEBUG_PRINTS = true;

// Queue item structure
typedef struct {
    uint8_t* data;
    uint64_t timestampData;
    size_t length;
    uint8_t charType; // 0=TxData, 1=Timestamp, 2=Alert
} QueueItem;

// Server context structure
typedef struct {
    BLEClient* pClient;
    BLERemoteCharacteristic* cmdChar;
    BLERemoteCharacteristic* txDataChar;
    BLERemoteCharacteristic* timestampChar;
    BLERemoteCharacteristic* alertChar;
    
    QueueHandle_t txDataQueue;
    QueueHandle_t timestampQueue;
    QueueHandle_t alertQueue;
    
    uint8_t currentCommand;
    uint8_t buffersReceived;
    bool connected;
    bool transferActive;
    bool transferCompleted;
    bool timestampCompleted;
} ServerContext;

ServerContext servers[NUMBER_OF_SERVERS];

// Shared variables
volatile bool readyForAlerts = false;
volatile bool reconstructionTriggered = false;
volatile bool transferActiveGlobal = false;
int64_t timeDiff[NUMBER_OF_SERVERS] = {0};
uint64_t clientAlertTime = 0;

// Data buffers
uint8_t receivedBuffer[sizeof(sendBuffer)];
sendBuffer transferDataBuffers[NUMBER_OF_SERVERS][NUMBER_OF_SEND_BUFFERS];
uint16_t receivedBytes[NUMBER_OF_SERVERS] = {0};

// Thresholds
int16_t currentThreshold[NUMBER_OF_SERVERS] = {0};
int16_t voltageThreshold[NUMBER_OF_SERVERS] = {0};
volatile bool thresholdsReady[NUMBER_OF_SERVERS] = {false};

// Combined buffer data
int combinedElements;
int firstBufferSkip;
int secondBufferSkip;
// int16_t combinedCurrentBuffer[NUMBER_OF_SERVERS][COMBINED_ELEMENTS] = {0};
// int16_t combinedVoltageBuffer[NUMBER_OF_SERVERS][COMBINED_ELEMENTS] = {0};

// Semaphores
SemaphoreHandle_t receiveBufferMutex[NUMBER_OF_SERVERS][NUMBER_OF_SEND_BUFFERS];
SemaphoreHandle_t dataTransferBufferMutex[NUMBER_OF_SERVERS];
SemaphoreHandle_t powerCalculationMutex;
SemaphoreHandle_t alertMutex;
SemaphoreHandle_t commandMutex[NUMBER_OF_SERVERS];
SemaphoreHandle_t combinedBufferMutex[NUMBER_OF_SERVERS];

// Timer
esp_timer_handle_t reconstructionTimer;

// Forward declarations
void calculatePowerFromArray(bool reconstructionCall);
void sendTransferRequestIdx(int serverIdx);
void serverWorkerTask(void* pvParameters);
void processTxData(int serverIdx, uint8_t* data, size_t length);
void processTimestamp(int serverIdx, uint8_t* data, size_t length);
void processAlert(int serverIdx, uint8_t* data, size_t length);
void setMtuSize(BLEClient* pclient);


void setMtuSize(BLEClient* pclient) {
    pclient->setMTU(START_MTU_SIZE);
    uint16_t mtu = pclient->getMTU();
    Serial.printf("Connected with MTU: %d\n", mtu);
    Serial.printf("[Client] Max possible chunk size: %d\n", mtu - 3);
}


class ClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) 
    {
        for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
        {
            if (pclient == servers[i].pClient) 
            {
                Serial.printf("Connected to Server %d\n", i);
                servers[i].connected = true;
                readyForAlerts = false;
                setMtuSize(pclient);
                break;
            }
        }
    }

    void onDisconnect(BLEClient* pclient) {
        for (int i = 0; i < NUMBER_OF_SERVERS; i++) {
            if (pclient == servers[i].pClient) {
                Serial.printf("Disconnected from Server %d\n", i);
                servers[i].timestampCompleted = false;
                servers[i].connected = false;
                readyForAlerts = false;
                xSemaphoreGive(alertMutex);
                break;
            }
        }
        esp_timer_stop(reconstructionTimer);
    }
};


void calculateCombinedDataValues()
{
    // Za bafer proslosti, alert je na mjestu SEND_BUFFER_ELEMENTS - 1, SEND_BUFFER_ELEMENTS ima 21 element, sto znaci da ima 5 perioda i alert.  PERIOD_ELEMENTS
    // Da bi smanjili gresku pri racunanju snage oko alert-a, potrebno je njega postaviti kao sredisnji element u slucaju neparnog broja PERIOD_ELEMENTS a u slucajno parnog broja na polovina - 1. Integer dijeljenje radi trunc tako da je flooring svakako.
    // Za bafer buducnosti, alert je na mjestu 0

    // Ukupno imam 2x SEND_BUFFER_ELEMENTS - 1 (jer se alert ponavlja 2x), trebam samo izracunati offset za koji trebam pomjeriti pocetak kopiranja i sa memcpy odraditi smjestanje podataka. Izi pizi lemon skvizi
    if (PERIOD_ELEMENTS % 2 == 1)
    {
        combinedElements = 2 * SEND_BUFFER_ELEMENTS - PERIOD_ELEMENTS - 3;
        firstBufferSkip = PERIOD_ELEMENTS / 2 + 1;
        secondBufferSkip = PERIOD_ELEMENTS / 2 + 2;
    }
    else
    {
        combinedElements = 2 * SEND_BUFFER_ELEMENTS - PERIOD_ELEMENTS - 2;
        firstBufferSkip = PERIOD_ELEMENTS / 2 + 1;
        secondBufferSkip = PERIOD_ELEMENTS / 2 + 1;

    }
}


void calculatePowerFromArray(bool reconstructionCall)
{
    if ((xSemaphoreTake(powerCalculationMutex, pdMS_TO_TICKS(1)) == pdTRUE))
    {
        // alociraj kombinovane nizove
        int16_t combinedCurrentBuffer[NUMBER_OF_SERVERS][combinedElements] = {0};
        int16_t combinedVoltageBuffer[NUMBER_OF_SERVERS][combinedElements] = {0};
        // prodji kroz sve nizove i popuni odgovarajuce vrijednosti.
        for (int serverIdx = 0; serverIdx < NUMBER_OF_SERVERS; serverIdx++)
        {
            if((xSemaphoreTake(combinedBufferMutex[serverIdx], pdMS_TO_TICKS(1)) == pdTRUE))
            {
                memset(&combinedCurrentBuffer[serverIdx][0], 0, sizeof(int16_t)*combinedElements);
                memset(&combinedVoltageBuffer[serverIdx][0], 0, sizeof(int16_t)*combinedElements);
                if(xSemaphoreTake(receiveBufferMutex[serverIdx][0], pdMS_TO_TICKS(1)) == pdTRUE)
                {
                    memcpy(&combinedCurrentBuffer[serverIdx][0], &transferDataBuffers[serverIdx][0].currentBuffer[firstBufferSkip], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - firstBufferSkip));
                    memcpy(&combinedVoltageBuffer[serverIdx][0], &transferDataBuffers[serverIdx][0].voltageBuffer[firstBufferSkip], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - firstBufferSkip));
                    if (!reconstructionCall && (serverIdx == 0))
                    {
                        Serial.printf("[Alert value] Current buffer alert value: %d, value before %d .\n",transferDataBuffers[serverIdx][0].currentBuffer[SEND_BUFFER_ELEMENTS - 1], transferDataBuffers[serverIdx][0].currentBuffer[SEND_BUFFER_ELEMENTS - 2]);
                        Serial.printf("[Alert value] Copied value (alert): %d, at idx %d Value before, value after %d.\n",combinedCurrentBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip - 1)], SEND_BUFFER_ELEMENTS - firstBufferSkip - 1, combinedCurrentBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip - 2)], combinedCurrentBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip)]);
                    }
                    
                    xSemaphoreGive(receiveBufferMutex[serverIdx][0]);
                }
                else
                {
                    Serial.printf("[Calculate power] Unable to fetch past buffer's mutex for Server %d .\n", serverIdx);
                }
                if(xSemaphoreTake(receiveBufferMutex[serverIdx][1], pdMS_TO_TICKS(1)) == pdTRUE)
                {
                    if (!reconstructionCall && (serverIdx == 0))
                    {
                        Serial.printf("[Alert value] Current buffer alert value: %d, value after %d .\n",transferDataBuffers[serverIdx][1].currentBuffer[0], transferDataBuffers[serverIdx][1].currentBuffer[1]);
                    }
                    memcpy(&combinedCurrentBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip)], &transferDataBuffers[serverIdx][1].currentBuffer[1], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - firstBufferSkip));
                    memcpy(&combinedVoltageBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip)], &transferDataBuffers[serverIdx][1].voltageBuffer[1], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - firstBufferSkip));
                    if (!reconstructionCall && (serverIdx == 0))
                    {
                        Serial.printf("[Alert value] Alert value: %d at idx %d.\n",combinedCurrentBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip - 1)], (SEND_BUFFER_ELEMENTS - firstBufferSkip - 1) , transferDataBuffers[serverIdx][1].currentBuffer[1]);
                        Serial.printf("[Alert value] Border value: %d at idx %d.\n",combinedCurrentBuffer[serverIdx][(SEND_BUFFER_ELEMENTS - firstBufferSkip)], (SEND_BUFFER_ELEMENTS - firstBufferSkip) , transferDataBuffers[serverIdx][1].currentBuffer[1]);
                    }
                    xSemaphoreGive(receiveBufferMutex[serverIdx][1]);
                }
                else
                {
                    Serial.printf("[Calculate power] Unable to fetch future buffer's mutex for Server %d.\n", serverIdx);
                }
                xSemaphoreGive(combinedBufferMutex[serverIdx]);
            }
            else
            {
                Serial.printf("[Calculate power] Unable to fetch combine buffer mutex for server %d.\n", serverIdx);
            }
            

        }

        // Sada imam podatke poredane tako da mogu pronaci max 
        int maxPowerPeriod[NUMBER_OF_SERVERS];
        float maxPowerValue[NUMBER_OF_SERVERS] = {-__FLT32_MAX__};
        int alertPeriod = (SEND_BUFFER_ELEMENTS - firstBufferSkip) / PERIOD_ELEMENTS;
        // Pronaci max struju i max napon u periodu. 
        // Pronaci razliku u njihovim indeksima i tako naci deltu
        // Pronaci fi preko delte kao 2 * PI * delta_t, pa cos(Fi)
        // In case of the reconstruction, we only need to find the thresholds for current and voltage in that period.
        // In case of the alert, for every period sampled.
        if (reconstructionCall)
        {
            // TODO: Dodati ovdje combined semafor isto :)  kao i gore
            Serial.println("[Reconstruction] Reconstructing period values for all 3 servers:");
            // izracunaj sve vrijednosti prvo, zatim isprintaj
            int alertPeriodIdx = alertPeriod * PERIOD_ELEMENTS;
            for (int serverIdx = 0; serverIdx < NUMBER_OF_SERVERS; serverIdx++)
            {
                if((xSemaphoreTake(combinedBufferMutex[serverIdx], pdMS_TO_TICKS(1)) == pdTRUE))
                {       
                    int16_t maxCurrent = (int16_t)abs(combinedCurrentBuffer[serverIdx][alertPeriodIdx]);
                    int16_t maxVoltage = (int16_t)abs(combinedVoltageBuffer[serverIdx][alertPeriodIdx]);
                    int maxVoltageIdx = alertPeriodIdx;
                    int maxCurrentIdx = alertPeriodIdx;
                    for (int i = alertPeriodIdx + 1; i < (alertPeriod + 1) * PERIOD_ELEMENTS; i++)
                    {
                        // In order to go to the end of the period, we need to put >= in calculating max indices. Just to be sure in our calculation.
                        // I can't use abs values since I don't know how I'm cutting values.
                        if ((int16_t)abs(combinedVoltageBuffer[serverIdx][i]) >= maxVoltage)
                        {
                            maxVoltage = (int16_t)abs(combinedVoltageBuffer[serverIdx][i]);
                            maxVoltageIdx = i;
                        }
                        if ((int16_t)abs(combinedCurrentBuffer[serverIdx][i]) >= maxCurrent)
                        {
                            maxCurrent = (int16_t)abs(combinedCurrentBuffer[serverIdx][i]);
                            maxCurrentIdx = i;
                        }
                    }
                    currentThreshold[serverIdx] = (int16_t)ceil(maxCurrent * 1.1); // 10% increment of the threshold
                    voltageThreshold[serverIdx] = (int16_t)ceil(maxVoltage * 1.1);
                    thresholdsReady[serverIdx] = true;
                    Serial.printf("[Calculate Power - Server %d] New current threshold: %d. New voltage threshold: %d.\n", serverIdx, currentThreshold[serverIdx], voltageThreshold[serverIdx]);
                    Serial.printf("[Calculate Power - Server %d] Max current: %d. Max voltage: %d\n", serverIdx, maxCurrent, maxVoltage);
                    // Kada se racuna Fi, posmatra se napon, ako je fi pozitivan onda napon prednjaci, ako je negativan napon kasni
                    // kosinus fi racunamo preko lookup tabele za vrijednosti ciji je ulaz delta t pomjeren
                    // ako je vrijednost negativna, napon se dogodio prije. Ako je vrijednost pozitivna napon kasni.
                    float power = (maxVoltage * maxCurrent)/2.f * COS_PHI_TABLE[maxVoltageIdx - maxCurrentIdx + PERIOD_ELEMENTS - 1];
                    Serial.printf("[Calculate Power - Server %d] Power for reconstruction period is: %.5f\n", serverIdx, power);
                    xSemaphoreGive(combinedBufferMutex[serverIdx]);
                }
                else
                {
                    Serial.printf("[Calculate power] Unable to fetch combine buffer mutex for server %d.\n", serverIdx);
                }
            }
            Serial.println("[Reconstruction] Reconstruction period server values:");
            Serial.println("[Reconstruction] -idx-+----[Server A]----+----[Server C]----+----[Server D]----+");
            for (int i = alertPeriodIdx; i < (alertPeriod + 1) * PERIOD_ELEMENTS; i++)
            {
                Serial.printf("[Reconstruction] %4d | %4d[V] %4d [A] | %4d[V] %4d[A] | %4d[V] %4d[A] |\n", 
                    i, combinedVoltageBuffer[0][i], combinedCurrentBuffer[0][i], combinedVoltageBuffer[1][i], combinedCurrentBuffer[1][i], combinedVoltageBuffer[2][i], combinedCurrentBuffer[2][i]);
            }
        }
        else
        {                
            Serial.println("[Power calculation] Alert triggered calculation:");
            // Ako je alert podignut, onda samo ispisi snage
            for (int periodCounter = 0; periodCounter < combinedElements; periodCounter+= PERIOD_ELEMENTS)
            {
                for (int serverIdx = 0; serverIdx < NUMBER_OF_SERVERS; serverIdx++)
                {
                    if((xSemaphoreTake(combinedBufferMutex[serverIdx], pdMS_TO_TICKS(1)) == pdTRUE))
                    {    
                        int16_t maxCurrent = (int16_t)abs(combinedCurrentBuffer[serverIdx][periodCounter]);
                        int16_t maxVoltage = (int16_t)abs(combinedVoltageBuffer[serverIdx][periodCounter]);
                        int maxVoltageIdx = periodCounter;
                        int maxCurrentIdx = periodCounter;
                        for (int i = 1; i < PERIOD_ELEMENTS; i++)
                        {
                            if ((int16_t)abs(combinedVoltageBuffer[serverIdx][periodCounter + i]) > maxVoltage)
                            {
                                maxVoltage = (int16_t)abs(combinedVoltageBuffer[serverIdx][periodCounter + i]);
                                maxVoltageIdx = periodCounter + i;
                            }
                            if ((int16_t)abs(combinedCurrentBuffer[serverIdx][periodCounter + i]) > maxCurrent)
                            {
                                maxCurrent = (int16_t)abs(combinedCurrentBuffer[serverIdx][periodCounter + i]);
                                maxCurrentIdx = periodCounter + i;
                            }
                        }
                        Serial.printf("[Calculate Power - Server %d] Period: %d - Max current: %d. Max voltage: %d\n", serverIdx, periodCounter/PERIOD_ELEMENTS + 1, maxCurrent, maxVoltage);
                        // Kada se racuna Fi, posmatra se napon, ako je fi pozitivan onda napon prednjaci, ako je negativan napon kasni
                        // kosinus fi racunamo preko lookup tabele za vrijednosti ciji je ulaz delta t pomjeren
                        // ako je vrijednost negativna, napon se dogodio prije. Ako je vrijednost pozitivna napon kasni.
                        float power = (maxVoltage * maxCurrent)/2.f * COS_PHI_TABLE[maxVoltageIdx - maxCurrentIdx + PERIOD_ELEMENTS - 1];
                        if (power > maxPowerValue[serverIdx])
                        {
                            maxPowerValue[serverIdx] = power;
                            maxPowerPeriod[serverIdx] = periodCounter;
                        }
                        xSemaphoreGive(combinedBufferMutex[serverIdx]);
                        // Serial.printf("[Calculate Power - Server %d] Power for period: %d/%d is: %.5f\n", serverIdx, periodCounter/PERIOD_ELEMENTS + 1, combinedElements/PERIOD_ELEMENTS, power);
                    }
                    else
                    {
                        Serial.printf("[Calculate power] Unable to fetch combine buffer mutex for server %d.\n", serverIdx);
                    }       
                }
            }
            for (int serverIdx = 0; serverIdx < NUMBER_OF_SERVERS; serverIdx++)
            {
                Serial.printf("[Calculate Power - Server %d] Max Power %.5f in period: %d/%d\n", serverIdx, maxPowerValue[serverIdx], maxPowerPeriod[serverIdx]/PERIOD_ELEMENTS + 1, combinedElements/PERIOD_ELEMENTS);
            }
        }
        xSemaphoreGive(powerCalculationMutex);
    }
    else
    {
        Serial.println("[Power calculation] Unable to take print mutex.");
    }
   
}

// Notification callbacks
void dataTransferNotifyCallback_A(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 0;
        if (xQueueSend(servers[0].txDataQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void dataTransferNotifyCallback_C(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 0;
        if (xQueueSend(servers[1].txDataQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void dataTransferNotifyCallback_D(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 0;
        if (xQueueSend(servers[2].txDataQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void timestampNotifyCallback_A(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    Serial.println("[Time Stamp - Server A] Entering timestamp notify.");
    item.data = (uint8_t*)malloc(length);
    memcpy(&item.timestampData, pData, sizeof(uint64_t));
    if (item.data) 
    {
        Serial.println("[Time Stamp - Server A] Entering if statement.");
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 1;
        if (xQueueSend(servers[0].timestampQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            Serial.println("[Time Stamp - Server A] Unsuccsessful send.");
            free(item.data);
        }
    }
}

void timestampNotifyCallback_C(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    Serial.println("[Time Stamp - Server C] Entering timestamp notify.");
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 1;
        if (xQueueSend(servers[1].timestampQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void timestampNotifyCallback_D(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) 
{
    QueueItem item;
    Serial.println("[Time Stamp - Server D] Entering timestamp notify.");
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 1;
        if (xQueueSend(servers[2].timestampQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void alertNotifyCallback_A(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 2;
        if (xQueueSend(servers[0].alertQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void alertNotifyCallback_C(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 2;
        if (xQueueSend(servers[1].alertQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void alertNotifyCallback_D(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    QueueItem item;
    item.data = (uint8_t*)malloc(length);
    if (item.data) {
        memcpy(item.data, pData, length);
        item.length = length;
        item.charType = 2;
        if (xQueueSend(servers[2].alertQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            free(item.data);
        }
    }
}

void powerReconstruction(void* arg) {
    Serial.println("[Power Reconstruction] In reconstruction callback.");
    if (readyForAlerts && xSemaphoreTake(alertMutex, pdMS_TO_TICKS(1)) == pdTRUE && !transferActiveGlobal) {
        clientAlertTime = esp_timer_get_time();
        reconstructionTriggered = true;
        transferActiveGlobal = true;
        readyForAlerts = false;
        xSemaphoreGive(alertMutex);
    }
    else
    {
        Serial.printf("[Power Reconstruction] Can't access mutex, or readyForAlerts = %d, transferActiveGlobal = %d .\n", readyForAlerts, transferActiveGlobal);
    }
}

bool connectToServer(int serverIdx, BLEAddress serverAddress) {
    const char* serverNames[] = {"A", "C", "D"};
    Serial.printf("[Connect to Server %s] Connecting...\n", serverNames[serverIdx]);
    
    if (servers[serverIdx].pClient) {
        if (servers[serverIdx].pClient->isConnected()) {
            servers[serverIdx].pClient->disconnect();
        }
        delete servers[serverIdx].pClient;
        servers[serverIdx].pClient = nullptr;
    }
    
    servers[serverIdx].pClient = BLEDevice::createClient();
    servers[serverIdx].pClient->setClientCallbacks(new ClientCallbacks());

    if (!servers[serverIdx].pClient->connect(serverAddress)) {
        Serial.printf("[Connect to Server %s] Connection failed\n", serverNames[serverIdx]);
        return false;
    }

    BLERemoteService* pRemoteService = servers[serverIdx].pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.printf("[Connect to Server %s] Failed to find service\n", serverNames[serverIdx]);
        servers[serverIdx].pClient->disconnect();
        return false;
    }

    // Get characteristics
    servers[serverIdx].timestampChar = pRemoteService->getCharacteristic(TIMESTAMP_UUID);
    servers[serverIdx].txDataChar = pRemoteService->getCharacteristic(TX_DATA_UUID);
    servers[serverIdx].cmdChar = pRemoteService->getCharacteristic(RX_CONTROL_UUID);
    servers[serverIdx].alertChar = pRemoteService->getCharacteristic(ALERT_MEAS_UUID);

    if (!servers[serverIdx].timestampChar || !servers[serverIdx].txDataChar || 
        !servers[serverIdx].cmdChar || !servers[serverIdx].alertChar) {
        Serial.printf("[Connect to Server %s] Failed to find characteristics\n", serverNames[serverIdx]);
        servers[serverIdx].pClient->disconnect();
        return false;
    }

    // Register callbacks based on server index
    switch (serverIdx) {
        case 0:
            servers[serverIdx].timestampChar->registerForNotify(timestampNotifyCallback_A);
            servers[serverIdx].txDataChar->registerForNotify(dataTransferNotifyCallback_A);
            servers[serverIdx].alertChar->registerForNotify(alertNotifyCallback_A);
            break;
        case 1:
            servers[serverIdx].timestampChar->registerForNotify(timestampNotifyCallback_C);
            servers[serverIdx].txDataChar->registerForNotify(dataTransferNotifyCallback_C);
            servers[serverIdx].alertChar->registerForNotify(alertNotifyCallback_C);
            break;
        case 2:
            servers[serverIdx].timestampChar->registerForNotify(timestampNotifyCallback_D);
            servers[serverIdx].txDataChar->registerForNotify(dataTransferNotifyCallback_D);
            servers[serverIdx].alertChar->registerForNotify(alertNotifyCallback_D);
            break;
    }

    Serial.printf("[Connect to Server %s] Connected and subscribed\n", serverNames[serverIdx]);
    return true;
}

void serverWorkerTask(void* pvParameters) {
    int serverIdx = (int)pvParameters;
    QueueItem item;
    Serial.printf("[Server worker %d] Worker set up.\n", serverIdx);

    while (1) {
        // Process TxData queue first
        if (xQueueReceive(servers[serverIdx].txDataQueue, &item, pdMS_TO_TICKS(1)) == pdTRUE) {
            processTxData(serverIdx, item.data, item.length);
            free(item.data);
            continue;
        }

        // Process Timestamp queue
        if (xQueueReceive(servers[serverIdx].timestampQueue, &item, pdMS_TO_TICKS(1)) == pdTRUE) {
            uint64_t timeStamp = item.timestampData;
            processTimestamp(serverIdx, item.data, item.length);
            free(item.data);
            continue;
        }

        // Process Alert queue
        if (xQueueReceive(servers[serverIdx].alertQueue, &item, pdMS_TO_TICKS(1)) == pdTRUE) {
            processAlert(serverIdx, item.data, item.length);
            free(item.data);
            continue;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void processTxData(int serverIdx, uint8_t* data, size_t length) 
{
    Serial.printf("[TxData - Server %d] Processing data.\n", serverIdx);
    if (xSemaphoreTake(dataTransferBufferMutex[serverIdx], pdMS_TO_TICKS(10)) == pdTRUE) 
    {
        memcpy(&receivedBuffer[receivedBytes[serverIdx]], data, length);
        receivedBytes[serverIdx] += length;

        if (receivedBytes[serverIdx] < sizeof(sendBuffer)) 
        {
            servers[serverIdx].currentCommand = 2;
            sendTransferRequestIdx(serverIdx);
        } 
        else 
        {
            if (xSemaphoreTake(receiveBufferMutex[serverIdx][servers[serverIdx].buffersReceived], pdMS_TO_TICKS(10)) == pdTRUE) 
            {
                memcpy(&transferDataBuffers[serverIdx][servers[serverIdx].buffersReceived], receivedBuffer, sizeof(sendBuffer));
                receivedBytes[serverIdx] = 0;
                servers[serverIdx].buffersReceived++;

                if (servers[serverIdx].buffersReceived < NUMBER_OF_SEND_BUFFERS) 
                {
                    servers[serverIdx].currentCommand = 3;
                    sendTransferRequestIdx(serverIdx);
                } 
                else 
                {
                    servers[serverIdx].currentCommand = 4;
                    servers[serverIdx].transferActive = false;
                    servers[serverIdx].transferCompleted = true;
                }
                //  sendTransferRequestIdx(serverIdx);
                xSemaphoreGive(receiveBufferMutex[serverIdx][servers[serverIdx].buffersReceived-1]);
            }
        }
        xSemaphoreGive(dataTransferBufferMutex[serverIdx]);
    }
    else
    {
        Serial.printf("[TxData - Server %d] Unable to take mutex for data transfer.\n", serverIdx);
    }
}

void processTimestamp(int serverIdx, uint8_t* data, size_t length) {
    Serial.printf("[Timestamp - Server %d] Entered timestamp! Data length = %d\n",serverIdx, length);
    if (length == sizeof(uint64_t)) 
    {
        uint64_t serverTimestamp = *((uint64_t*)data);
        uint64_t clientTime = esp_timer_get_time();
        timeDiff[serverIdx] = clientTime - serverTimestamp;

        if (!servers[serverIdx].timestampCompleted) 
        {
            servers[serverIdx].timestampCompleted = true;
            if (DEBUG_PRINTS) {
                Serial.printf("[Server %d] First timestamp completed. Δ: %lld μs\n", serverIdx, timeDiff[serverIdx]);
            }
        }
    }
}

void processAlert(int serverIdx, uint8_t* data, size_t length) 
{
    Serial.printf("[Server %d Alert] Processing alert.\n", serverIdx);
    if (readyForAlerts && xSemaphoreTake(alertMutex, pdMS_TO_TICKS(1)) == pdTRUE && 
        length == sizeof(uint64_t) && !transferActiveGlobal) 
    {
        uint64_t alertTime = *((uint64_t*)data);
        clientAlertTime = timeDiff[serverIdx] + alertTime;
        Serial.printf("[Server %d Alert] Server alert time: %llu μs | Client calc alert time: %lld μs\n",
                    serverIdx, alertTime, clientAlertTime);
        Serial.printf("[Server %d A Alert] Requesting data transfer for calculated server time: %llu\n", serverIdx, alertTime);
        transferActiveGlobal = true; // U starom kodu je readyForTransfer
        readyForAlerts = false;
        xSemaphoreGive(alertMutex);
    }
    else
    {
        Serial.printf("[Server %d Alert] Unable to collect mutex. readyForAlerts=%d transferActiveGlobal=%d \n", serverIdx, readyForAlerts, transferActiveGlobal);
    }
}

void sendTransferRequestIdx(int serverIdx) {
    DATA_REQUEST request;
    request.command = servers[serverIdx].currentCommand;
    request.alertTimeStamp = clientAlertTime - timeDiff[serverIdx];
    request.reconstructionTriggered = (uint8_t)reconstructionTriggered;
    if (reconstructionTriggered)
    {
        request.currentThresholdValue = thresholdsReady[serverIdx] ? currentThreshold[serverIdx] : DEFAULT_THRESHOLD_CURRENT;
        request.voltageThresholdValue = thresholdsReady[serverIdx] ? voltageThreshold[serverIdx] : DEFAULT_THRESHOLD_VOLTAGE;
    }

    if (serverIdx >= 0 && serverIdx < NUMBER_OF_SERVERS) {
        if (xSemaphoreTake(commandMutex[serverIdx], pdMS_TO_TICKS(10)) == pdTRUE) {
            servers[serverIdx].cmdChar->writeValue((uint8_t*)&request, sizeof(DATA_REQUEST), true);
            xSemaphoreGive(commandMutex[serverIdx]);
        }
    } else {
        // Send to all servers
        for (int i = 0; i < NUMBER_OF_SERVERS; i++) {
            if (xSemaphoreTake(commandMutex[i], pdMS_TO_TICKS(10)) == pdTRUE) {
                servers[i].cmdChar->writeValue((uint8_t*)&servers[i].currentCommand, sizeof(uint8_t), true);
                xSemaphoreGive(commandMutex[i]);
            }
        }
    }
}


void printTaskPriorities() {
    TaskHandle_t bleTask = xTaskGetHandle("BT");         // BLE stack task
    TaskHandle_t callbackTask = xTaskGetHandle("loop");  // Arduino loop

    Serial.printf("BLE Stack Priority: %d\n", uxTaskPriorityGet(bleTask));
    Serial.printf("Arduino Loop Priority: %d\n", uxTaskPriorityGet(callbackTask));
}


void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 BLE Multi-Server Client");

    // Initialize buffers
    memset(receivedBuffer, 0, sizeof(sendBuffer));
    memset(transferDataBuffers, 0, sizeof(transferDataBuffers));
    memset(receivedBytes, 0, sizeof(receivedBytes));

    // Initialize semaphores
    for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
    {
        for (int j = 0; j < NUMBER_OF_SEND_BUFFERS; j++) 
        {
            receiveBufferMutex[i][j] = xSemaphoreCreateMutex();
        }
        dataTransferBufferMutex[i] = xSemaphoreCreateMutex();
        commandMutex[i] = xSemaphoreCreateMutex();
        combinedBufferMutex[i] = xSemaphoreCreateMutex();
        thresholdsReady[i] = false;
        
        // Initialize server contexts
        servers[i].buffersReceived = 0;
        servers[i].transferActive = false;
        servers[i].transferCompleted = false;
        servers[i].timestampCompleted = false;
        
        // Create queues
        servers[i].txDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueueItem));
        if (servers[i].txDataQueue == NULL) {
            Serial.println("TxData queue not created!");
        }
        servers[i].timestampQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueueItem));
        if (servers[i].timestampQueue == NULL) {
            Serial.println("Time stamp queue not created!");
        }
        servers[i].alertQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueueItem));
        if (servers[i].alertQueue == NULL) {
            Serial.println("Alert queue not created!");
        }    
    }

    powerCalculationMutex = xSemaphoreCreateMutex();
    alertMutex = xSemaphoreCreateMutex();

    // Initialize BLE
    BLEDevice::init("ESP32_Multi_Server_Client");

    // Connect to servers
    while (!connectToServer(0, serverAddress_A)) {
        Serial.println("Retrying Server A in 2 seconds...");
        delay(2000);
    }
    while (!connectToServer(1, serverAddress_C)) {
        Serial.println("Retrying Server C in 2 seconds...");
        delay(2000);
    }
    while (!connectToServer(2, serverAddress_D)) {
        Serial.println("Retrying Server D in 2 seconds...");
        delay(2000);
    }

    // Create worker tasks
    // for (int i = 0; i < NUMBER_OF_SERVERS; i++) {
    //     xTaskCreate(serverWorkerTask, "ServerWorker_A", 8192, (void*)i, 1, NULL);
    // }

    xTaskCreate(serverWorkerTask, "ServerWorker_1", 4096, (void*)0, 1, NULL);
    xTaskCreate(serverWorkerTask, "ServerWorker_2", 4096, (void*)1, 1, NULL);
    xTaskCreate(serverWorkerTask, "ServerWorker_3", 4096, (void*)2, 1, NULL);

    // Setup reconstruction timer
    esp_timer_create_args_t timerArgs = {
        .callback = &powerReconstruction,
        .name = "reconstructionTimer"
    };
    esp_timer_create(&timerArgs, &reconstructionTimer);

    printTaskPriorities();
    calculateCombinedDataValues();
    Serial.printf("[Setup] combinedElements = %d, firstBufferSkip = %d, secondBufferSkip = %d.\n",combinedElements, firstBufferSkip, secondBufferSkip);

    // poredaj booleane
    readyForAlerts = false;
    Serial.println("Setup complete");
}

void loop() {
    // Handle reconnections
    for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
    {
        if (!servers[i].pClient || !servers[i].pClient->isConnected()) 
        {
            const BLEAddress addresses[] = {serverAddress_A, serverAddress_C, serverAddress_D};
            if (connectToServer(i, addresses[i])) 
            {
                Serial.printf("Reconnected to Server %d\n", i);
            } else 
            {
                delay(2000);
            }
        }
    }

    // Check if all servers are ready for transfer
    // bool allReady = true;
    bool allTimestampsComplete = true;
    for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
    {
        // if (!servers[i].transferActive)
        //  {
        //     allReady = false;
        // }
        if (!servers[i].timestampCompleted) 
        {
            allTimestampsComplete = false;
        }
    }

    // Startati samo ako su svi konektovani
    if (servers[0].connected && servers[1].connected && servers[2].connected)
    {
        // Start reconstruction timer if all timestamps are complete
        if (allTimestampsComplete && !esp_timer_is_active(reconstructionTimer)) 
        {
            Serial.println("[Loop] Timestamps from client, server A, server C and server D all synced. Starting reconstruction timer.");
            readyForAlerts = true;
            esp_timer_start_periodic(reconstructionTimer, RECONSTRUCTION_TIME_US);
        }

        // Start transfer if all servers are ready
        if (transferActiveGlobal) // U starom kodu je readyForTransfer
        {
            // readyForAlerts = false; // Blokiraj alerte
            transferActiveGlobal = false;
            Serial.printf("[Loop] Filling all servers - command 1.\n"); 
            for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
            {
                servers[i].transferActive = true;
                servers[i].transferCompleted = false;
                servers[i].currentCommand = 1;
                servers[i].buffersReceived = 0;
                thresholdsReady[i] = false;
                sendTransferRequestIdx(i);
            }
        }

        // Ovo nam u sustini sad i ne treba, jer processTxData ce da obavi sva slanja za komande 2 i 3
        // if (allReady) // svi transferi su u toku
        // {
        //     Serial.printf("[Loop] Every transferActive for servers is set to true.\n");
        //     for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
        //     {
        //         sendTransferRequestIdx(i);
        //     }
        // }

        // Check if all transfers are complete
        bool allComplete = true;
        for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
        {
            if (!servers[i].transferCompleted) 
            {
                allComplete = false;
                break;
            }
        }

        // Process data when all transfers are complete
        if (allComplete && !readyForAlerts) 
        {
            Serial.printf("[Loop] First 3 steps completed.\n");
            Serial.printf("[Loop] Calculating power.\n");
            calculatePowerFromArray(reconstructionTriggered);
            Serial.printf("[Loop] Sending final command.\n");
            for (int i = 0; i < NUMBER_OF_SERVERS; i++) 
            {
                sendTransferRequestIdx(i);
            }
            // calculatePowerFromArray(reconstructionTriggered);
            reconstructionTriggered = false;
            readyForAlerts = true;
        }
    }

    delay(10);
}