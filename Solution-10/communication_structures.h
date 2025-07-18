#ifndef COMMUNICATION_STRUCTURES_H
#define COMMUNICATION_STRUCTURES_H

#include <stdint.h>

#define NUMBER_OF_STORE_BUFFERS (3) // Three buffers to store 30 seconds of data.
#define NUMBER_OF_SEND_BUFFERS  (2) // One for future samples one for past samples.
#define SAMPLING_FREQUENCY      (100) // Frekvencija samplovanja, probati do 1kHz doci

#define ALERT_TIME_S            (0.1)
#define TIME_STORED_S           (1) // 1 sekunda se cuva u jednom store bufferu
#define STORE_BUFFER_ELEMENTS    (int)(TIME_STORED_S * SAMPLING_FREQUENCY)
#define STORE_BUFFER_SIZE       (STORE_BUFFER_ELEMENTS * sizeof(uint16_t))
// #define SEND_BUFFER_SIZE        (ALERT_TIME_S * SAMPLING_FREQUENCY * sizeof(uint16_t)) // 100ms 1 bafer
#define SEND_BUFFER_ELEMENTS    (int)(ALERT_TIME_S * SAMPLING_FREQUENCY + 1) // (alert + 100 ms)
#define SEND_BUFFER_SIZE        (SEND_BUFFER_ELEMENTS * sizeof(uint16_t)) // 100ms 1 bafer

typedef struct storeBuffer 
{
    uint16_t    sampleIdx;
    uint64_t    startTimeStamp;
    uint16_t    currentBuffer[STORE_BUFFER_ELEMENTS];
    uint16_t    voltageBuffer[STORE_BUFFER_ELEMENTS];
} STORE_BUFFER;

struct sendBuffer
{
    uint64_t    timeStamp;
    uint16_t    currentBuffer[SEND_BUFFER_ELEMENTS];
    uint16_t    voltageBuffer[SEND_BUFFER_ELEMENTS];
};

/**
 * @brief Data transfer struct.
 */
typedef struct dataRequest
{
    //! 1 - Client sends out alert time; 2 - Request next chunk; 3 - Start of future transfer; 4 - End of all transfer;
    uint8_t     command; 
    // Time on server at which alert happened calculated by the client (scaling parameter, for multiple servers). 
    uint64_t    alertTimeStamp;
} DATA_REQUEST;


#endif // COMMUNICATION_STRUCTURES_H