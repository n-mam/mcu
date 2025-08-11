#ifndef MS5837_H
#define MS5837_H

#include <mcl/mcl.h>
#include <mcl/i2c.h>

struct MS5837 {

    static const float Pa;
    static const float bar;
    static const float mbar;

    static const uint8_t MS5837_30BA = 0;
    static const uint8_t MS5837_02BA = 1;
    static const uint8_t MS5837_UNKNOWN = 255;

	const uint8_t MS5837_02BA01 = 0x00; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
	const uint8_t MS5837_02BA21 = 0x15; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
	const uint8_t MS5837_30BA26 = 0x1A; // Sensor version: From MS5837_30BA datasheet Version PROM Word 0

    MS5837(int sda, int scl, uint freq) :
        _model(MS5837_UNKNOWN), fluidDensity(1029) {
        #if defined (STM32)
        i2c::initialize(sda, scl, freq, I2C1, GPIOB);
        #elif defined (PICO)
        i2c::initialize(sda, scl, freq, i2c0);
        #endif
    }

    bool init() {
        // Reset the MS5837
        i2c::write(MS5837_ADDR, MS5837_RESET, NULL, 0);
        // Wait for reset to complete
        mcl::sleep_ms(100);
        // Read calibration values and CRC
        for (uint8_t i = 0; i < 7; i++) {
            uint8_t buf[2];
            i2c::read(MS5837_ADDR, MS5837_PROM_READ + i * 2, buf, 2);
            C[i] = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
        }
        // Verify data with CRC
        uint8_t crcRead = C[0] >> 12;
        uint8_t crcCalculated = crc4(C);
        if (crcCalculated != crcRead) {
            return false;
        }
        LOG << "crc check successful";
        // Set model based on sensor version
        uint8_t version = (C[0] >> 5) & 0x7F;
        if (version == MS5837_02BA01 || version == MS5837_02BA21) {
            _model = MS5837_02BA;
        } else if (version == MS5837_30BA26) {
            _model = MS5837_30BA;
        } else {
            _model = MS5837_UNKNOWN;
        }
        return true;
    }

    void setModel(uint8_t model) {
        _model = model;
    }

    uint8_t getModel() {
        return _model;
    }

    void setFluidDensity(float density) {
        fluidDensity = density;
    }

    void read() {
        // Request D1 conversion
        i2c::write(MS5837_ADDR, MS5837_CONVERT_D1_8192, NULL, 0);
        // Max conversion time
        mcl::sleep_ms(20);

        uint8_t buf[3];
        i2c::read(MS5837_ADDR, MS5837_ADC_READ, buf, 3);
        D1_pres = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

        // Request D2 conversion
        i2c::write(MS5837_ADDR, MS5837_CONVERT_D2_8192, NULL, 0);
        // Max conversion time
        mcl::sleep_ms(20);

        memset(buf, 0, 3);
        i2c::read(MS5837_ADDR, MS5837_ADC_READ, buf, 3);
        D2_temp = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

        calculate();
    }

    float pressure(float conversion = 1.0f) {
        return (_model == MS5837_02BA) ? P * conversion / 100.0f : P * conversion / 10.0f;
    }

    float temperature() {
        return TEMP / 100.0f;
    }

    float depth() {
        return (pressure(MS5837::Pa) - 101300) / (fluidDensity * 9.80665);
    }

    float altitude() {
        return (1 - pow((pressure() / 1013.25), .190284)) * 145366.45 * .3048;
    }

	private:

    int32_t P;
    int32_t TEMP;
    uint16_t C[8];
    uint8_t _model;
    float fluidDensity;
    uint32_t D1_pres, D2_temp;

    void calculate() {
        int32_t dT = D2_temp - uint32_t(C[5]) * 256l;
        int64_t SENS, OFF;
        if (_model == MS5837_02BA) {
            SENS = int64_t(C[1]) * 65536l + (int64_t(C[3]) * dT) / 128l;
            OFF = int64_t(C[2]) * 131072l + (int64_t(C[4]) * dT) / 64l;
            P = (D1_pres * SENS / 2097152l - OFF) / 32768l;
        } else {
            SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
            OFF = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;
            P = (D1_pres * SENS / 2097152l - OFF) / 8192l;
        }
        TEMP = 2000l + int64_t(dT) * C[6] / 8388608LL;

        // Second order compensation omitted for brevity
        // Add your second order compensation logic here
    }

    uint8_t crc4(uint16_t n_prom[]) {
        uint16_t n_rem = 0;
        n_prom[0] &= 0x0FFF;
        n_prom[7] = 0;

        for (uint8_t i = 0; i < 16; i++) {
            if (i % 2 == 1) {
                n_rem ^= (n_prom[i >> 1] & 0x00FF);
            } else {
                n_rem ^= (n_prom[i >> 1] >> 8);
            }
            for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
                n_rem = (n_rem & 0x8000) ? (n_rem << 1) ^ 0x3000 : (n_rem << 1);
            }
        }
        return ((n_rem >> 12) & 0x000F) ^ 0x00;
    }

    static const uint8_t MS5837_ADDR = 0x76;
    static const uint8_t MS5837_RESET = 0x1E;
    static const uint8_t MS5837_ADC_READ = 0x00;
    static const uint8_t MS5837_PROM_READ = 0xA0;
    static const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
    static const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;
};

const float MS5837::Pa = 100.0f;
const float MS5837::mbar = 1.0f;
const float MS5837::bar = 0.001f;

#endif
