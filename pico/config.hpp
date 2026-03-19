#pragma once

//stablization loop speed
#define AUV_STB_LOOP_MS 10

//BNO055
#define BNO055_PORT i2c1
#define BNO055_ADDR 0x28
#define BNO055_SDA  27
#define BNO055_SCL  26

//PIO of ESC
#define PIO_VB 5
#define PIO_VR 6
#define PIO_VL 7
#define PIO_HR 8
#define PIO_HL 9

//RASPI UART 
#define RP_TX 29
#define RP_RX 28
#define RP_BAUDRATE  115200
#define RP_UARTID  uart0