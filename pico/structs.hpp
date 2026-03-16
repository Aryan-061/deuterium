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
    //Eulers for reference
    float roll;
    float pitch;
} State;

typedef struct {
    uint16_t VB, VR, VL, HR, HL;
    int zoffset;
} Throttle;