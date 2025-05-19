#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "esp_bt.h"
#include "../constants.h"
#include "../communication_structures.h"
#include "../power_signal_data_long.h"
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
volatile bool thresholdsReady = false;
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

// Data transfer
uint8_t receivedBuffer[sizeof(sendBuffer)]; // Primamo uint8 podatak koji kasnije moramo kastovati
sendBuffer readData;
sendBuffer readDataBuffers[NUMBER_OF_SEND_BUFFERS];
uint8_t buffersReceived = 0;
uint16_t receivedBytes = 0;
volatile bool transferActive = false;
volatile bool sendCommand = false;

int16_t currentThreshold;
int16_t voltageThreshold;

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
                Serial.printf("%d: %d [A] - %d [V]\n",
                    i, readDataBuffers[sendingBufferIdx].currentBuffer[i], readDataBuffers[sendingBufferIdx].voltageBuffer[i]);
            }
            xSemaphoreGive(receiveBufferMutex[sendingBufferIdx]);
        }
        else
        {
            Serial.println("[Print power] Couldn't access lock in print hex values.");
        }
    }
}


// PERIOD_ELEMENTS
void calculatePowerFromArray(bool reconstructionCall)
{
    // Za bafer proslosti, alert je na mjestu SEND_BUFFER_ELEMENTS - 1, SEND_BUFFER_ELEMENTS ima 21 element, sto znaci da ima 5 perioda i alert.  PERIOD_ELEMENTS
    // Da bi smanjili gresku pri racunanju snage oko alert-a, potrebno je njega postaviti kao sredisnji element u slucaju neparnog broja PERIOD_ELEMENTS a u slucajno parnog broja na polovina - 1. Integer dijeljenje radi trunc tako da je flooring svakako.
    // Za bafer buducnosti, alert je na mjestu 0

    // Ukupno imam 2x SEND_BUFFER_ELEMENTS - 1 (jer se alert ponavlja 2x), trebam samo izracunati offset za koji trebam pomjeriti pocetak kopiranja i sa memcpy odraditi smjestanje podataka. Izi pizi lemon skvizi
    int combinedElements;
    int firstBufferSkip;
    int secondBufferSkip;
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
        secondBufferSkip = PERIOD_ELEMENTS / 2 + 2;

    }
    int16_t combinedCurrentBuffer[combinedElements] = {0};
    int16_t combinedVoltageBuffer[combinedElements] = {0};
    if(xSemaphoreTake(receiveBufferMutex[0], 5) == pdTRUE)
    {
        memcpy(combinedCurrentBuffer, &readDataBuffers[0].currentBuffer[firstBufferSkip], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - firstBufferSkip));
        memcpy(combinedVoltageBuffer, &readDataBuffers[0].voltageBuffer[firstBufferSkip], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - firstBufferSkip));
        xSemaphoreGive(receiveBufferMutex[0]);
    }
    else
    {
        Serial.println("[Calculate power] Unable to fetch past buffer's mutex.");
    }
    if(xSemaphoreTake(receiveBufferMutex[1], 5) == pdTRUE)
    {
        memcpy(&combinedCurrentBuffer[(SEND_BUFFER_ELEMENTS - firstBufferSkip)], &readDataBuffers[1].currentBuffer[1], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - secondBufferSkip));
        memcpy(&combinedVoltageBuffer[(SEND_BUFFER_ELEMENTS - firstBufferSkip)], &readDataBuffers[1].voltageBuffer[1], sizeof(int16_t) * (SEND_BUFFER_ELEMENTS - secondBufferSkip));
        xSemaphoreGive(receiveBufferMutex[1]);
    }
    else
    {
        Serial.println("[Calculate power] Unable to fetch future buffer's mutex.");
    }
    // Sada imam podatke poredane tako da mogu pronaci max 
    int alertPeriod = (SEND_BUFFER_ELEMENTS - firstBufferSkip) / PERIOD_ELEMENTS;
    // Pronaci max struju i max napon u periodu. 
    // Pronaci razliku u njihovim indeksima i tako naci deltu
    // Pronaci fi preko delte kao 2 * PI * delta_t, pa cos(Fi)
    // In case of the reconstruction, we only need to find the thresholds for current and voltage in that period.
    // In case of the alert, for every period sampled.
    if (reconstructionCall)
    {
        // TODO: Dodati ovdje combined semafor isto :)  kao i gore
        Serial.println("[Reconstruction] Reconstructing period values:");
        Serial.printf("%d: %d [V] - %d [A]\n", alertPeriod * PERIOD_ELEMENTS, combinedVoltageBuffer[alertPeriod * PERIOD_ELEMENTS], combinedCurrentBuffer[alertPeriod * PERIOD_ELEMENTS]);
        int16_t maxCurrent = combinedCurrentBuffer[alertPeriod * PERIOD_ELEMENTS];
        int16_t maxVoltage = combinedVoltageBuffer[alertPeriod * PERIOD_ELEMENTS];
        int maxVoltageIdx = alertPeriod * PERIOD_ELEMENTS;
        int maxCurrentIdx = alertPeriod * PERIOD_ELEMENTS;
        for (int i = (alertPeriod * PERIOD_ELEMENTS) + 1; i < (alertPeriod + 1) * PERIOD_ELEMENTS; i++)
        {
            // In order to go to the end of the period, we need to put >= in calculating max indices. Just to be sure in our calculation.
            // I can't use abs values since I don't know how I'm cutting values.
            if (combinedVoltageBuffer[i] >= maxVoltage)
            {
                maxVoltage = combinedVoltageBuffer[i];
                maxVoltageIdx = i;
            }
            if (combinedCurrentBuffer[i] >= maxCurrent)
            {
                maxCurrent = combinedCurrentBuffer[i];
                maxCurrentIdx = i;
            }
            Serial.printf("%d: %d [V] - %d [A]\n", i, combinedVoltageBuffer[i], combinedCurrentBuffer[i]);
        }
        currentThreshold = maxCurrent > 0 ? (int16_t)ceil(maxCurrent * 1.1) : DEFAULT_THRESHOLD_CURRENT; // 10% increment of the threshold
        voltageThreshold = maxVoltage > 0 ? (int16_t)ceil(maxVoltage * 1.1) : DEFAULT_THRESHOLD_VOLTAGE;
        thresholdsReady = true;
        Serial.printf("[Calculate Power] New current threshold: %d. New voltage threshold: %d.\n", currentThreshold, voltageThreshold);
        Serial.printf("[Calculate Power] Max current: %d. Max voltage: %d\n", maxCurrent, maxVoltage);
        // Kada se racuna Fi, posmatra se napon, ako je fi pozitivan onda napon prednjaci, ako je negativan napon kasni
        // kosinus fi racunamo preko lookup tabele za vrijednosti ciji je ulaz delta t pomjeren
        // ako je vrijednost negativna, napon se dogodio prije. Ako je vrijednost pozitivna napon kasni.
        float power = (maxVoltage * maxCurrent)/2.f * COS_PHI_TABLE[maxVoltageIdx - maxCurrentIdx + PERIOD_ELEMENTS - 1];
        Serial.printf("[Calculate Power] Power for reconstruction period is: %.5f\n", power);
    }
    else
    {
        for (int periodCounter = 0; periodCounter < combinedElements; periodCounter+= PERIOD_ELEMENTS)
        {
            int16_t maxCurrent = combinedCurrentBuffer[periodCounter];
            int16_t maxVoltage = combinedVoltageBuffer[periodCounter];
            int maxVoltageIdx = periodCounter;
            int maxCurrentIdx = periodCounter;
            for (int i = 1; i < PERIOD_ELEMENTS; i++)
            {
                if (combinedVoltageBuffer[periodCounter + i] > maxVoltage)
                {
                    maxVoltage = combinedVoltageBuffer[periodCounter + i];
                    maxVoltageIdx = periodCounter + i;
                }
                if (combinedCurrentBuffer[periodCounter + i] > maxCurrent)
                {
                    maxCurrent = combinedCurrentBuffer[periodCounter + i];
                    maxCurrentIdx = periodCounter + i;
                }
            }
            Serial.printf("[Calculate Power] Max current: %d. Max voltage: %d\n", maxCurrent, maxVoltage);
            // Kada se racuna Fi, posmatra se napon, ako je fi pozitivan onda napon prednjaci, ako je negativan napon kasni
            // kosinus fi racunamo preko lookup tabele za vrijednosti ciji je ulaz delta t pomjeren
            // ako je vrijednost negativna, napon se dogodio prije. Ako je vrijednost pozitivna napon kasni.
            float power = (maxVoltage * maxCurrent)/2.f * COS_PHI_TABLE[maxVoltageIdx - maxCurrentIdx + PERIOD_ELEMENTS - 1];
            Serial.printf("[Calculate Power] Power for period: %d/%d is: %.5f\n",periodCounter/PERIOD_ELEMENTS + 1, combinedElements/PERIOD_ELEMENTS, power);
        }
        Serial.printf("[Calculate Power] Alert done. Resetting thresholds to default.\n");
        voltageThreshold = DEFAULT_THRESHOLD_VOLTAGE;
        currentThreshold = DEFAULT_THRESHOLD_CURRENT;
        thresholdsReady = true;
    }
}


void powerReconstruction(void* arg)
{
    // Ukljuci se samo ukoliko je transfer inactive
    Serial.println("[Reconstruction] Entered reconstruction callback.");
    if (!transferActive)
    {
        Serial.println("[Reconstruction] Callback active.");
        uint64_t clientTime = esp_timer_get_time();
        serverAlertTimeStamp = clientTime - timeDiff;
        reconstructionTriggered = true;
        readyForTransfer = true;
    }
    else
    {
        Serial.println("[Reconstruction] Callback inactive. Transfer in progress.");
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

// Timestamp notify
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
                Serial.println("[Reconstruction] Starting reconstruction timer.");
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
        Serial.printf("[Write command] Request command: %d, alert time sent: %llu\n", request.command, request.alertTimeStamp);
    }
}


bool connectToServer() 
{
    Serial.print("[Connect to server] Connecting to ");
    Serial.println(serverAddress.toString().c_str());
    if (pClient != nullptr) 
    {
        if (pClient->isConnected()) 
        {
            pClient->disconnect();
        }
        delete pClient;
        pClient = nullptr;
        Serial.println("[Connect to server] Disconnecting and setting client to nullptr.");
    }
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    // Connect to the server
    // if (!pClient->connect(serverAddress, BLE_ADDR_TYPE_PUBLIC , 1000)) {
    if (!pClient->connect(serverAddress)) 
    {
        Serial.println("[Connect to server] Connection failed");
        return false;
    }

    // Get the service
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
        Serial.println("[Connect to server] Failed to find service");
        pClient->disconnect();
        return false;
    }
    Serial.println("[Connect to server] Service discovered successfully"); // Add this

    // Get the timestamp characteristic
    pTimestampChar = pRemoteService->getCharacteristic(TIMESTAMP_UUID);
    if (pTimestampChar == nullptr) {
        Serial.println("[Connect to server] Failed to find timestamp characteristic");
        pClient->disconnect();
        return false;
    }
    delay(100);


    // delay(100);
    // Get TX data characteristic (This will read data)
    pTxData = pRemoteService->getCharacteristic(TX_DATA_UUID);
    if (pTxData == nullptr) {
        Serial.println("[Connect to server] Failed to find transfer data characteristic");
        pClient->disconnect();
        return false;
    }

    delay(100);
    // Get RX Controll characteristic. (Sending out commands to a server)
    pRxControl = pRemoteService->getCharacteristic(RX_CONTROL_UUID);
    if (pRxControl == nullptr) {
        Serial.println("[Connect to server] Failed to find control characteristic");
        pClient->disconnect();
        return false;
    }

    delay(100);
    // Get the alert characteristic
    pAlertMeasurement = pRemoteService->getCharacteristic(ALERT_MEAS_UUID);
    if (pAlertMeasurement == nullptr) {
        Serial.println("[Connect to server] Failed to find alert characteristic");
        Serial.println("[Connect to server] Available characteristics:");
        std::map<std::string, BLERemoteCharacteristic*>* pChars = pRemoteService->getCharacteristics();
        for(auto it = pChars->begin(); it != pChars->end(); it++) {
            Serial.println(it->first.c_str());
        }
        pClient->disconnect();
        return false;
    }
    
    // Subscribe to notifications
    if(pTimestampChar->canNotify()) {
        Serial.println("[Connect to server] Registered for timestamp notify.");
        pTimestampChar->registerForNotify(timestampNotifyCallback);
    }

    // Subscribe to notifications of the alert
    if(pAlertMeasurement->canNotify()) {
        Serial.println("[Connect to server] Registered for alert notify.");
        pAlertMeasurement->registerForNotify(alertNotifyCallback);
    }

    if(pRxControl->canWrite() && pRxControl->canWriteNoResponse())
    {
        Serial.println("[Connect to server] Registered for write on control.");
    }

    // Subscribe to notifications for data transfer
    if(pTxData->canNotify()) {
        Serial.println("[Connect to server] Registered for data transfer notify.");
        pTxData->registerForNotify(dataTransferNotifyCallback);
    }
    
    Serial.println("[Connect to server] Connected and subscribed to timestamp updates and latency tests.");
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
            Serial.println("[Setup] Receive mutex creation failed!");
            while(1); // Critical failure
        }
    }
    if (DEBUG_PRINTS)
    {
        Serial.println("[Setup] Receive mutex creation finished.");
    }

    // Initialize BLE
    BLEDevice::init("ESP32_Client_Alert_Data_Transmission"); // Using client MAC in name

    String bleMac = BLEDevice::getAddress().toString().c_str();
    Serial.println("[Setup] BLE MAC: " + bleMac);
    
    // Attempt connection
    while (!connectToServer()) {
        Serial.println("[Setup] Retrying in 2 seconds...");
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
        Serial.println("[Setup] Created reconstruction timer.");
    }
}

void loop() {
    if (!connected && disconnected) {
        Serial.println("[Loop] Attempting reconnection...");
        if (connectToServer()) 
        {
            Serial.println("[Loop] Reconnected successfully");
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
            Serial.println("[Loop] Entering the transfer loop!");
            readyForTransfer = false;
            transferActive = true;
            receivedBytes = 0;
            buffersReceived = 0;
            request.command = 1;
            request.alertTimeStamp = serverAlertTimeStamp;
            request.reconstructionTriggered = 0;
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
                Serial.printf("[Reconstruction] Add new threshold values: %d [V] %d [A].\n", voltageThreshold, currentThreshold);
                request.reconstructionTriggered = 1;
                calculatePowerFromArray(true);
                reconstructionTriggered = false;
                sendCommand = false;
                transferActive = false;
                if (thresholdsReady)
                {
                    request.currentThresholdValue = currentThreshold;
                    request.voltageThresholdValue = voltageThreshold;
                    thresholdsReady = false;
                }
                sendTransferRequest();
                // printPowerValues();
            }
            else
            {
                calculatePowerFromArray(false);
                if (thresholdsReady)
                {
                    request.currentThresholdValue = currentThreshold;
                    request.voltageThresholdValue = voltageThreshold;
                    thresholdsReady = false;
                }
                sendCommand = false;
                transferActive = false;
                sendTransferRequest();
                // printPowerValues();
            }
            

        }
    }

    delay(50); // Prevent watchdog triggers
}