#include "imu.hpp"

extern State state;

static const float alpha = 0.1f;
static float wx_filt_last = 0.0f;
static float wy_filt_last = 0.0f;
static float wz_filt_last = 0.0f;

uint8_t buffer[8];

void imu::init() {

    i2c_init(BNO055_PORT, 400 * 1000);
    gpio_set_function(BNO055_SDA, GPIO_FUNC_I2C);
    gpio_set_function(BNO055_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(BNO055_SDA);
    gpio_pull_up(BNO055_SCL);

    uint8_t reg = 0x00;
    uint8_t chipID = 0;

    while (chipID != 0xA0) {
        printf("BNO055 not connected\n");
        i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg, 1, true);
        i2c_read_blocking(BNO055_PORT, BNO055_ADDR, &chipID, 1, false);
        sleep_ms(500);
    }

    printf("BNO055 connected\n");
    sleep_ms(1000);

    uint8_t data[2];

    /* CONFIG MODE */
    data[0] = 0x3D;
    data[1] = 0x00;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(50);

    /* Internal oscillator */
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(50);

    /* Units: gyro rad/s */
    data[0] = 0x3B;
    data[1] = 0x06;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(50);

    /* NDOF_FMC_OFF */
    data[0] = 0x3D;
    data[1] = 0x0B;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(500);

    printf("IMU running in NDOF_FMC_OFF (quaternion output)\n");
}

void imu::update() {

    /* -------- Quaternion -------- */
    uint8_t reg_quat = 0x20;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg_quat, 1, true);
    i2c_read_blocking(BNO055_PORT, BNO055_ADDR, buffer, 8, false);

    int16_t qw_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t qx_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t qy_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t qz_raw = (int16_t)((buffer[7] << 8) | buffer[6]);

    state.qw = qw_raw / 16384.0f;
    state.qx = qx_raw / 16384.0f;
    state.qy = qy_raw / 16384.0f;
    state.qz = qz_raw / 16384.0f;

    /* -------- Angular velocity -------- */
    uint8_t reg_gyro = 0x14;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg_gyro, 1, true);
    i2c_read_blocking(BNO055_PORT, BNO055_ADDR, buffer, 6, false);

    int16_t raw_wx = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_wy = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_wz = (int16_t)((buffer[5] << 8) | buffer[4]);

    state.wx = raw_wx / 900.0f;
    state.wy = raw_wy / 900.0f;
    state.wz = raw_wz / 900.0f;

    /* -------- Low-pass filter -------- */
    state.wx = alpha * state.wx + (1.0f - alpha) * wx_filt_last;
    state.wy = alpha * state.wy + (1.0f - alpha) * wy_filt_last;
    state.wz = alpha * state.wz + (1.0f - alpha) * wz_filt_last;

    wx_filt_last = state.wx;
    wy_filt_last = state.wy;
    wz_filt_last = state.wz;
}