#ifndef MPU6050
#define MPU6050

#include <mcl/i2c.h>

namespace imu {

struct mpu6050 {

    // accl offsets
    float ax_offset = 0;
    float ay_offset = 0;
    float az_offset = 0;
    // gyro offsets
    float gx_offset = 0;
    float gy_offset = 0;
    float gz_offset = 0;

    static constexpr int MPU6050_I2C_ADDR = 0x68;
    static constexpr float ACC_SCALE_INV = 1.0 / 16384.0;
    static constexpr float GYRO_SCALE_INV = 1.0 / 131.0;

    template <typename T>
    using XYZ = std::tuple<T, T, T, T, T, T>;

    mpu6050(int sda, int scl, int freq = 400'000) {
        #if defined (STM32)
        i2c::initialize(sda, scl, freq, I2C1, GPIOB);
        #elif defined (PICO)
        i2c::initialize(sda, scl, freq, i2c0);
        #endif
    }

    void initialize() {
        // wake up MPU6050
        uint8_t buf = 0x00;
        i2c::write(MPU6050_I2C_ADDR, 0x6B, &buf, 1);
        // Accel ±2g
        i2c::write(MPU6050_I2C_ADDR, 0x1C, &buf, 1);
        // Gyro ±250 deg/s
        i2c::write(MPU6050_I2C_ADDR, 0x1B, &buf, 1);
        // DLPF ~44Hz (stable output)
        buf = 0x03;
        i2c::write(MPU6050_I2C_ADDR, 0x1A, &buf, 1);
        mcl::sleep_ms(50);        
    }

    void calibrate() {
        LOG << "calibrating...";
        LOG << "please keep the sensor still.";
        int actual = 0;
        const int samples = 1024;
        int32_t a[3] = {0, 0, 0};
        int32_t g[3] = {0, 0, 0};
        for (int i = 0; i < samples; i++) {
            auto [ax, ay, az, gx, gy, gz] = 
                read_raw();
            ++actual;
            a[0] += ax;
            a[1] += ay;
            a[2] += az;
            g[0] += gx;
            g[1] += gy;
            g[2] += gz;
            mcl::sleep_ms(5);
        }
        if (actual < (samples / 2)) return;
        // at rest, the desired raw accel values
        // should be: ax = 0, ay = 0, az = 16384
        ax_offset = (a[0] / (float)actual) - 0;
        ay_offset = (a[1] / (float)actual) - 0;
        az_offset = (a[2] / (float)actual) - 16384.0;
        // at rest gyro should be all 0's
        gx_offset = (g[0] / (float)actual) - 0;
        gy_offset = (g[1] / (float)actual) - 0;
        gz_offset = (g[2] / (float)actual) - 0;
        LOG << "Calibration done !";
        LOG << "ax_offset = " << ax_offset;
        LOG << "ay_offset = " << ay_offset;
        LOG << "az_offset = " << az_offset;
        LOG << "gx_offset = " << gx_offset;
        LOG << "gy_offset = " << gy_offset;
        LOG << "gz_offset = " << gz_offset;
        LOG << "valid samples = " << actual;
    }
    
    auto read_raw() -> XYZ<int16_t> {
        uint8_t b[14];
        int16_t ax, ay, az;
        int16_t gx, gy, gz;        
        i2c::read(MPU6050_I2C_ADDR, 0x3B, b, 14);
        ax = (int16_t)((b[0] << 8) | b[1]);
        ay = (int16_t)((b[2] << 8) | b[3]);
        az = (int16_t)((b[4] << 8) | b[5]);
        // skip temp (b[6], b[7])
        gx = (int16_t)((b[8] << 8) | b[9]);
        gy = (int16_t)((b[10] << 8) | b[11]);
        gz = (int16_t)((b[12] << 8) | b[13]);
        return {ax, ay, az, gx, gy, gz};        
    }

    auto read_calibrated() -> XYZ<float> {
        auto [ax, ay, az, gx, gy, gz] = read_raw();
        // offset correction and 
        // scale to physical units
        return {
            (ax - ax_offset) * ACC_SCALE_INV,
            (ay - ay_offset) * ACC_SCALE_INV,
            (az - az_offset) * ACC_SCALE_INV,
            (gx - gx_offset) * GYRO_SCALE_INV,
            (gy - gy_offset) * GYRO_SCALE_INV,
            (gz - gz_offset) * GYRO_SCALE_INV};
    }
};

} //namespace imu

#endif