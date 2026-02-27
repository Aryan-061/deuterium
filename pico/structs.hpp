#pragma once
#include <stdint.h>

typedef struct {
    // Orientation (quaternion)
    float qw;
    float qx;
    float qy;
    float qz;

    // Angular velocity (body frame)
    float wx;
    float wy;
    float wz;
} State;

typedef struct {
    uint16_t VB, VR, VL, HR, HL;
    int zoffset;
} Throttle;