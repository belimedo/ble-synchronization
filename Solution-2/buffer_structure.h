#ifndef BUFFER_STRUCTURE_H
#define BUFFER_STRUCTURE_H

#include <stdint.h>

#define SAMPLING_FREQUENCY  (10) // Frekvencija samplovanja
#define MAX_BUFFER_SIZE     (256) // Velicina bafera bez gubitka podataka
#define TIME_COVERED        (MAX_BUFFER_SIZE / sizeof(uint8_t) / SAMPLING_FREQUENCY) // Vrijeme koje jedan bafer prenosi
#define TRUE_BUFFER_SIZE    (TIME_COVERED * sizeof(uint8_t) / SAMPLING_FREQUENCY ) // Velicina bafera koja ce se prenositi
#define NUMBER_OF_BUFFERS   (3) // Trenutni broj, a moglo bi se i izracunati na osnovu vremena koje donosi jedan bafer pa tako kontrolisati i broj bafera kao WANTED_TIME / TIME_COVERED
#define TIMEOUT_PERIOD      (1.f / SAMPLING_FREQUENCY)


typedef struct buffer_structure
{
    uint64_t time_stamp;
    uint8_t buffer_values[TRUE_BUFFER_SIZE];
} BUFFER;

BUFFER buffers[NUMBER_OF_BUFFERS]; 


#endif