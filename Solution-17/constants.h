#ifndef CONSTANTS_H
#define CONSTANTS_H
// List of MACs
#define ESP_A_MAC           "EC:94:CB:4C:B8:76"
#define ESP_B_MAC           "EC:94:CB:4A:7C:26"
#define ESP_C_MAC           "88:13:BF:00:7E:06"
#define ESP_D_MAC           "C0:5D:89:B0:75:BA"

// UUIDs (must match server)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914a"
#define TX_DATA_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (buffer data)
#define RX_CONTROL_UUID     "2c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Write (control)
#define TIMESTAMP_UUID      "3c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (timestamp)
#define LATENCY_UUID        "4c95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify Write (latency)
#define ALERT_MEAS_UUID     "ac95d5e3-d8f7-413a-bf3d-7a2e5d7ab87c" // Notify (incorrect data)

#define DEFAULT_CHUNK_SIZE  (20)
#define START_MTU_SIZE      (247)

#define DEFAULT_THRESHOLD_VOLTAGE   (330) // 0xFF01
#define DEFAULT_THRESHOLD_CURRENT   (11) // 0xFF01

#endif