#ifndef BNO85_H
#define BNO85_H

#include <mcl/mcl.h>
#include <mcl/i2c.h>
#include <sns/imu/bno.h>

extern "C" {
    #include <sns/imu/sh2/euler.h>
    #include <sns/imu/sh2/sh2_err.h>
    #include <sns/imu/sh2/sh2_SensorValue.h>
}

#include <math.h>
#include <cstring>

namespace imu {

bool reset_occurred = false;
sh2_SensorValue_t sensor_value;
sh2_SensorEvent_t sensor_event;
u8 accStatus, gyroStatus, magStatus;
constexpr uint8_t BNO085_I2C_ADDR = 0x4A;

bool hal_reset();
void readAdvertisement();
uint32_t getTimeUs(sh2_Hal_t *);
void hal_callback(void *, sh2_AsyncEvent_t *);
void sensorHandler(void *, sh2_SensorEvent_t *);

int bno85_i2c_open(sh2_Hal_t *);
void bno85_i2c_close(sh2_Hal_t *);
int bno85_i2c_write(sh2_Hal_t *, uint8_t *, unsigned);
int bno85_i2c_read(sh2_Hal_t *, uint8_t *, unsigned , uint32_t *);

double _i, _j, _k, _r;
double _yaw, _pitch, _roll;

struct bno85 : public bno {

    sh2_Hal_t _sh2_hal;
    sh2_ProductIds_t _prodIds;

    bno85(int sda, int scl, uint freq) : bno(sda, scl, freq) {}

    bool init_i2c_hal() {
        if (!hal_reset()) return false;
        readAdvertisement();
        _sh2_hal.open = bno85_i2c_open;
        _sh2_hal.read = bno85_i2c_read;
        _sh2_hal.close = bno85_i2c_close;
        _sh2_hal.write = bno85_i2c_write;
        _sh2_hal.getTimeUs = getTimeUs;
        auto status = sh2_open(&_sh2_hal, hal_callback, NULL);
        if (status != SH2_OK) {
            LOG << "sh2_open failed";
            return false;
        }
        readProductIds();
        status = sh2_setSensorCallback(sensorHandler, NULL);
        if (status != SH2_OK) {
            LOG << "sh2_setSensorCallback failed";
            return false;
        }
        return true;
    }

    virtual auto getEulerAngles() -> YPR {
        sh2_service();
        if (hasReset()) {
            enableCalibration();
            enableReports();
        }
        return {_yaw, _pitch, _roll};
    }

    virtual auto getQuaternion() -> QAT {
        return {_i, _j, _k, _r};
    }

    virtual auto getCalibrationStatus() -> CST {
        return {0, 0, 0, 0};
    }

    bool readProductIds() {
        memset(&_prodIds, 0, sizeof(_prodIds));
        auto status = sh2_getProdIds(&_prodIds);
        if (status != SH2_OK) {
            LOG << "sh2_getProdIds failed";
            return false;
        }
        for (int n = 0; n < _prodIds.numEntries; n++) { // sh2.h line 60
            LOG << "Part: " << _prodIds.entry[n].swPartNumber;
            LOG << "Version: " << _prodIds.entry[n].swVersionMajor << "." <<
                _prodIds.entry[n].swVersionMinor << "." << _prodIds.entry[n].swVersionPatch;
            LOG << "Build: " << _prodIds.entry[n].swBuildNumber;
        }
        return true;
    }

    bool enableCalibration() {
        auto status = sh2_setCalConfig(SH2_CAL_ACCEL|SH2_CAL_GYRO|SH2_CAL_MAG);
        LOG << "sh2_setCalConfig " << status;
        return (status == SH2_OK);
    }

    bool enableReports() {
        if (!enableReport(SH2_ACCELEROMETER, 5)) {
            return false;
        }
        if (!enableReport(SH2_ROTATION_VECTOR, 10)) {
            return false;
        }
        if (!enableReport(SH2_GYROSCOPE_CALIBRATED, 5)) {
            return false;
        }
        if (!enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 5)) {
            return false;
        }
        return true;
    }

    bool enableReport(sh2_SensorId_t sensorId, float frequency) {
        uint32_t interval_us = (uint32_t)(1000000 / frequency);
        static sh2_SensorConfig_t config;
        config.sensorSpecific = 0;
        config.batchInterval_us = 0;
        config.wakeupEnabled = false;
        config.changeSensitivity = 0;
        config.alwaysOnEnabled = false;
        config.reportInterval_us = interval_us;
        config.changeSensitivityEnabled = false;
        config.changeSensitivityRelative = false;
        auto status = sh2_setSensorConfig(sensorId, &config);
        if (status != SH2_OK) {
            LOG << "sh2_setSensorConfig failed " << sensorId;
        }
        return (status == SH2_OK);
    }

    bool hasReset(void) {
        bool x = reset_occurred;
        reset_occurred = false;
        if (x) LOG << "imu was reset";
        return x;
    }
};

inline void readAdvertisement() {
    uint8_t shtp_header[4];
    i2c::read(BNO085_I2C_ADDR, 0, shtp_header, 4, false);
    uint16_t len = (uint16_t)shtp_header[1] << 8 | (uint16_t)shtp_header[0];
    len &= 0x7FFF;
    uint8_t advertisement[512];
    i2c::read(BNO085_I2C_ADDR, 0, advertisement, len, false);
}

inline bool hal_reset() {
    for (int i = 0; i < 2; i++) {
        sh2_devReset();
        mcl::sleep_ms(30);
    }
    mcl::sleep_ms(300);
    return true;
}

// callback for reset events and other non-sensor events received from SH-2 sensor hub
inline void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        imu::reset_occurred = true;
        LOG << "hal_callback SH2_RESET";
    }
}

inline void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
    sensor_event = *event;
    auto status = sh2_decodeSensorEvent(&sensor_value, &sensor_event);
    if (status != SH2_OK) {
        LOG << "sh2_decodeSensorEvent failed";
        return;
    }
    bool cal_event = false;
    switch (sensor_event.reportId) {
        // The absolute rotation vector provides an orientation output that
        // is expressed as a quaternion referenced to magnetic north and gravity game
        // rotation vector output aligns the quaternion output to an arbitrary orientation
        case SH2_ROTATION_VECTOR: {
            _i = sensor_value.un.rotationVector.i;
            _j = sensor_value.un.rotationVector.j;
            _k = sensor_value.un.rotationVector.k;
            _r = sensor_value.un.rotationVector.real;
            _roll = q_to_roll(_r, _i, _j, _k);
            _pitch = q_to_pitch(_r, _i, _j, _k);
            _yaw = q_to_yaw(_r, _i, _j, _k) * (180.0 / M_PI);
            // LOG << "{" << "\"i\":" << _i << ", "
            //     << "\"j\":" << _j << ", "
            //     << "\"k\":" << _k << ", "
            //     << "\"r\":" << _r << ", "
            //     << "\"yaw\":" << _yaw << "}\n";
            break;
        }
        case SH2_ACCELEROMETER:
            cal_event = true;
            accStatus = (sensor_event.report[2] & 0x03);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            cal_event = true;
            gyroStatus = (sensor_event.report[2] & 0x03);
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            cal_event = true;
            magStatus = (sensor_event.report[2] & 0x03);
            break;
    }
    if (cal_event) {
        // LOG << "{" <<
        //     "\"cal_gyro\":" << unsigned(gyroStatus) << ", " <<
        //     "\"cal_acc\":" << unsigned(accStatus) << ", " <<
        //     "\"cal_mag\":" << unsigned(magStatus) << "}\n";
    }
}

inline int bno85_i2c_open(sh2_Hal_t *self) {
    LOG << "i2c_open";
    if (!hal_reset()) return 1;
    return 0;
}

inline void bno85_i2c_close(sh2_Hal_t *self) {
    LOG << "i2c_close";
}

inline int bno85_i2c_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *t_us) {
    *t_us = getTimeUs(self);
    uint8_t header[4];
    i2c::read(BNO085_I2C_ADDR, 0, header, 4, false);
    // Determine amount to read
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    // Unset the "continue" bit
    packet_size &= ~0x8000;
    size_t i2c_buffer_max = 512;
    if (packet_size > len) {
        // packet wouldn't fit in our buffer
        return 0;
    }
    // the number of non-header bytes to read
    uint16_t cargo_remaining = packet_size;
    uint8_t i2c_buffer[i2c_buffer_max];
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;

    while (cargo_remaining > 0) {
        if (first_read) {
            read_size = std::min(i2c_buffer_max, (size_t)cargo_remaining);
        } else {
            read_size = std::min(i2c_buffer_max, (size_t)cargo_remaining + 4);
        }
        i2c::read(BNO085_I2C_ADDR, 0, i2c_buffer, read_size, false);
        if (first_read) {
            // The first time we're saving the "original" header, so include it in the
            // cargo count
            cargo_read_amount = read_size;
            memcpy(buffer, i2c_buffer, cargo_read_amount);
            first_read = false;
        } else {
            // this is not the first read, so copy from 4 bytes after the beginning of
            // the i2c buffer to skip the header included with every new i2c read and
            // don't include the header in the amount of cargo read
            cargo_read_amount = read_size - 4;
            memcpy(buffer, i2c_buffer + 4, cargo_read_amount);
        }
        // advance our pointer by the amount of cargo read
        buffer += cargo_read_amount;
        // mark the cargo as received
        cargo_remaining -= cargo_read_amount;
    }
    return packet_size;
}

inline int bno85_i2c_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len) {
    uint16_t length = (len > SH2_HAL_MAX_TRANSFER_OUT) ? SH2_HAL_MAX_TRANSFER_OUT : len;
    i2c::write(BNO085_I2C_ADDR, 0, buffer, length, false);
    return length;
}

inline uint32_t getTimeUs(sh2_Hal_t *self) {
    #if defined (PICO)
    return to_us_since_boot(get_absolute_time());
    #elif defined (STM32)
    return mcl::get_tick() * 1000;
    #endif
}

}

#endif