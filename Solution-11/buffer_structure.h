#ifndef BUFFER_STRUCTURE_H
#define BUFFER_STRUCTURE_H

#include <stdint.h>

#define SAMPLING_FREQUENCY  (100) // Frekvencija samplovanja
#define MAX_BUFFER_SIZE     (256) // Velicina bafera bez gubitka podataka
#define TIME_COVERED        (MAX_BUFFER_SIZE / sizeof(uint8_t) / SAMPLING_FREQUENCY) // Vrijeme koje jedan bafer prenosi
#define TRUE_BUFFER_SIZE    (TIME_COVERED * sizeof(uint8_t) / SAMPLING_FREQUENCY ) // Velicina bafera koja ce se prenositi
#define NUMBER_OF_BUFFERS   (3) // Trenutni broj, a moglo bi se i izracunati na osnovu vremena koje donosi jedan bafer pa tako kontrolisati i broj bafera kao WANTED_TIME / TIME_COVERED
#define TIMEOUT_PERIOD      (1.f / SAMPLING_FREQUENCY)

#define ALERT_TIME_S        (0.1)
#define TIME_STORED_S       (1) // 1 sekunda se cuva u jednom store bufferu
#define STORE_BUFFER_SIZE   (TIME_STORED_S * SAMPLING_FREQUENCY * sizeof(uint16_t))
#define SEND_BUFFER_SIZE    (ALERT_TIME_S * SAMPLING_FREQUENCY * sizeof(uint16_t)) // 100ms 1 bafer


typedef struct buffer_structure
{
    uint64_t time_stamp;
    uint8_t buffer_values[TRUE_BUFFER_SIZE];
} BUFFER;

BUFFER buffers[NUMBER_OF_BUFFERS]; 


struct storeBuffer 
{
    uint16_t    storeBufferIdx;
    uint64_t    startTimeStamp;
    uint16_t    buffer[STORE_BUFFER_SIZE];
};

struct sendBuffer
{
    uint64_t    timeStamp;
    uint8_t     *dataBufferPtr; // TODO: Ovdje ce biti send_buffer_size podataka, koji se odredjuje uz pomoc MTU-a
};

struct transferBuffer
{
    bool        transferReady;
    bool        transferCompleted;
    sendBuffer  sendBuffer;
};
#endif