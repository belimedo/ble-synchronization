#ifndef COMMUNICATION_STRUCTURES_H
#define COMMUNICATION_STRUCTURES_H

#include <stdint.h>

#define NUMBER_OF_STORE_BUFFERS (3) // Three buffers to store 30 seconds of data.
#define NUMBER_OF_SEND_BUFFERS  (2) // One for future samples one for past samples.
#define SAMPLING_FREQUENCY      (1000) // Frekvencija samplovanja, probati do 1kHz doci
#define POWER_GRID_FREQUENCY    (50) // frekvencija mreze
#define PERIOD_ELEMENTS         (SAMPLING_FREQUENCY/POWER_GRID_FREQUENCY) // TODO: Ovo bi se trebalo izracunati i hardkodovati

#define ALERT_TIME_S            (0.1)
#define TIME_STORED_S           (1) // 1 sekunda se cuva u jednom store bufferu
#define STORE_BUFFER_ELEMENTS    (int)(TIME_STORED_S * SAMPLING_FREQUENCY)
#define STORE_BUFFER_SIZE       (STORE_BUFFER_ELEMENTS * sizeof(int16_t))
// #define SEND_BUFFER_SIZE        (ALERT_TIME_S * SAMPLING_FREQUENCY * sizeof(int16_t)) // 100ms 1 bafer
#define SEND_BUFFER_ELEMENTS    (int)(ALERT_TIME_S * SAMPLING_FREQUENCY + 1) // (alert + 100 ms)
#define SEND_BUFFER_SIZE        (SEND_BUFFER_ELEMENTS * sizeof(int16_t)) // 100ms 1 bafer

// #define SAMPLE_INTERVAL_US      (10000) // 10 ms, 10 000 us 100 Hz
// #define SAMPLE_INTERVAL_US      (5000) // 5 ms, 5 000 us 200 Hz
// #define SAMPLE_INTERVAL_US      (2000) // 2 ms, 2 000 us 500 Hz
#define SAMPLE_INTERVAL_US      (1000) // 1 ms, 1 000 us 1000 Hz
#define TIMESYNC_INTERVAL_US    (60000000) // 60 s, 60 000 000 us
#define RECONSTRUCTION_TIME_US          (60000000) // 60 seconds 60 * 1'000'000

typedef struct storeBuffer 
{
    uint16_t    sampleIdx;
    uint64_t    startTimeStamp;
    int16_t    currentBuffer[STORE_BUFFER_ELEMENTS];
    int16_t    voltageBuffer[STORE_BUFFER_ELEMENTS];
} STORE_BUFFER;

struct sendBuffer
{
    uint64_t    timeStamp;
    int16_t    currentBuffer[SEND_BUFFER_ELEMENTS];
    int16_t    voltageBuffer[SEND_BUFFER_ELEMENTS];
};

/**
 * @brief Data transfer struct.
 */
typedef struct dataRequest
{
    //! 1 - Client sends out alert time; 2 - Request next chunk; 3 - Start of future transfer; 4 - End of all transfer;
    uint8_t     command;
    uint8_t     reconstructionTriggered;
    int16_t    currentThresholdValue;
    int16_t    voltageThresholdValue;
    // Time on server at which alert happened calculated by the client (scaling parameter, for multiple servers). 
    uint64_t    alertTimeStamp;
} DATA_REQUEST;


#endif // COMMUNICATION_STRUCTURES_H