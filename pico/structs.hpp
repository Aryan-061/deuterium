#include <stdint.h>
#pragma once

typedef struct {
    float roll, pitch, yaw;
    float wx, wy, wz;
} State;

typedef struct {
    uint16_t VB, VR, VL, HR, HL;
    int zoffset;
} Throttle;

typedef struct {
    uint8_t sof, dx, dy, dz, dyaw, crc0, crc1;
} RaspiRecFrame;