#ifndef COMMUNICATION_STRUCTURES_H
#define COMMUNICATION_STRUCTURES_H

#include <stdint.h>

#define NUMBER_OF_STORE_BUFFERS (3) // Three buffers to store 30 seconds of data.
#define NUMBER_OF_SEND_BUFFERS  (1) // One for future samples one for past samples.
#define SAMPLING_FREQUENCY      (5) // Frekvencija samplovanja 
#define TIME_STORED             (30) // 30 sekundi se cuva u jednom store bufferu
#define TIME_SENT               (5)
#define STORE_BUFFER_SIZE       (TIME_STORED * SAMPLING_FREQUENCY * sizeof(uint8_t))
#define SEND_BUFFER_SIZE        ((TIME_SENT * SAMPLING_FREQUENCY + 1) * sizeof(uint8_t)) // Add one extra time slice, to be on the safe side

typedef struct storeBuffer 
{
    uint16_t    sampleIdx;
    uint64_t    startTimeStamp;
    uint8_t     buffer[STORE_BUFFER_SIZE];
} STORE_BUFFER;

struct sendBuffer
{
    uint64_t    timeStamp;
    uint8_t     dataBufferPtr[SEND_BUFFER_SIZE];
};

typedef struct transferBuffer
{
    bool        transferReady;
    bool        transferCompleted;
    sendBuffer  previousSamples;
    sendBuffer  futureSamples;
} TRANSFER_BUFFER;


/**
 * @brief Data transfer struct.
 */
typedef struct dataRequest
{
    //! 1 - Client sends out alert time; 2 - Client parsed previous data transfer; 3 - End of transfer;
    uint8_t     command; 
    // How much time we need to transfer.
    uint8_t     bufferSizeInSeconds; 
    // Fallback mechanism for MTU sizes.
    uint16_t    chunkSize;
    // Time on server at which alert happened calculated by the client (scaling parameter, for multiple servers). 
    uint64_t    alertTimeStamp;
} DATA_REQUEST;


#endif // COMMUNICATION_STRUCTURES_H