#ifndef BNO55_H
#define BNO55_H

#include <mcl/mcl.h>
#include <sns/imu/bno.h>

extern "C" {
    #include <sns/imu/bno055/bno055.h>
}

#include <cstring>

namespace imu {

int8_t bno55_i2c_read(uint8_t, uint8_t, uint8_t *, uint8_t);
int8_t bno55_i2c_write(uint8_t, uint8_t, uint8_t *, uint8_t);

struct bno55 : public bno {

    struct bno055_t _bno;

    bno55(int sda, int scl, uint freq = 400*1000) : bno(sda, scl, freq) {
        _bno.bus_read = bno55_i2c_read;
        _bno.bus_write = bno55_i2c_write;
        _bno.delay_msec = mcl::sleep_ms;
        _bno.dev_addr = BNO055_I2C_ADDR1; // check jumper
        bno055_init(&_bno);
        mcl::sleep_ms(100);
        auto failed = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
        if (!failed) {
            mcl::sleep_ms(100);
            failed = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
            if (failed) {
                LOG << "bno055_set_operation_mode failed";
            }
            mcl::sleep_ms(100);
        } else {
            LOG << "bno055_set_power_mode failed";
        }
    }

    virtual auto getEulerAngles() -> YPR {
        bno055_euler_float_t euler;
        bno055_convert_float_euler_hpr_deg(&euler);
        return {euler.h, euler.p, euler.r};
    }

    virtual auto getQuaternion() -> QAT {
        // bno055_read_quaternion_wxyz();
        return {0, 0, 0, 0};
    }

    virtual auto getCalibrationStatus() -> CST {
        u8 sys;
        bno055_get_sys_calib_stat(&sys);
        u8 gyro;
        bno055_get_gyro_calib_stat(&gyro);
        u8 accl;
        bno055_get_accel_calib_stat(&accl);
        u8 mag;
        bno055_get_mag_calib_stat(&mag);
        return {gyro, accl, mag, sys};
    }
};

inline int8_t bno55_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    return i2c::read(dev_addr, reg_addr, reg_data, len);
}

inline int8_t bno55_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    return i2c::write(dev_addr, reg_addr, reg_data, len);
}

}

#endif