#ifndef HMC5883L_H
#define HMC5883L_H

#include <map>
#include <cmath>
#include <tuple>

#if defined (PICO)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#endif

namespace imu {

constexpr uint8_t HMC5883L_ADDR = 0x1E;
constexpr uint8_t HMC5883L_CONFIG_REG_A = 0x00;
constexpr uint8_t HMC5883L_CONFIG_REG_B = 0x01;
constexpr uint8_t DATA_OUTPUT_X_MSB_REG = 0x03;
constexpr uint8_t HMC5883L_CONFIG_REG_MODE = 0x02;

struct HMC5883L {

    double x_bias = -19.725;
    double y_bias = -6.375;
    double z_bias = 0.0;
    double x_scale = 0.7287274013172435;
    double y_scale = 0.6434063146391913;
    double z_scale = 0.0;

    double _gain = 1.9;
    double _decl = 0.0;
    serial::i2c *_i2c;

    std::map<double, std::pair<uint8_t, double>>
        _gain_map = {
            {0.88, {0 << 5, 0.73}},
            {1.3, {1 << 5, 0.92}},
            {1.9,  {2 << 5, 1.22}},
            {2.5,  {3 << 5, 1.52}},
            {4.0,  {4 << 5, 2.27}},
            {4.7,  {5 << 5, 2.56}},
            {5.6,  {6 << 5, 3.03}},
            {8.1,  {7 << 5, 4.35}}
        };

    HMC5883L(serial::i2c& bus, double gain = 1.9) : _gain(gain), _i2c(&bus) {
        uint8_t buf1[2] = {HMC5883L_CONFIG_REG_A, 0b01110000};
        _i2c->write(HMC5883L_ADDR, buf1[0], buf1 + 1, 1);
        uint8_t buf2[2] = {HMC5883L_CONFIG_REG_B, _gain_map[_gain].first};  // 1.9 gauss gain
        _i2c->write(HMC5883L_ADDR, buf2[0], buf2 + 1, 1);
        uint8_t buf3[2] = {HMC5883L_CONFIG_REG_MODE, 0x00};
        _i2c->write(HMC5883L_ADDR, buf3[0], buf3 + 1, 1);
        mcl::sleep_ms(12);
    }

    auto readMagRaw() {
        uint8_t buf[6] = { 0 };
        _i2c->read(HMC5883L_ADDR, DATA_OUTPUT_X_MSB_REG, buf, 6);
        int16_t raw_x = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
        int16_t raw_y = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
        int16_t raw_z = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
        LOG << "x_raw " << raw_x << ", y_raw " << raw_y << ", z_raw " << raw_z;
        // if (x & (1 << 15)) x = x - (1 << 16);
        // if (y & (1 << 15)) y = y - (1 << 16);
        // if (z & (1 << 15)) z = z - (1 << 16);
        double x = raw_x * _gain_map[_gain].second;
        double y = raw_y * _gain_map[_gain].second;
        double z = raw_z * _gain_map[_gain].second;
        return std::make_tuple(x, y, z);
    }

    auto readMagnetometer() {
        auto [x_raw, y_raw, z_raw] = readMagRaw();
        double x = (x_raw - x_bias) * x_scale;
        double y = (y_raw - y_bias) * y_scale;
        double z = (z_raw - z_bias) * z_scale;
        return std::make_tuple(x, y, z);
    }

    auto getHeading() {
        auto [x, y, z] = readMagnetometer();
        auto heading = std::atan2(y, x);
        heading += _decl;
        if (heading < 0) {
            heading += 2 * M_PI;
        } else if (heading > (2 * M_PI)) {
            heading -= 2 * M_PI;
        }
        heading = heading * (180 / M_PI); //degrees
        return heading;
    }

    auto calibrate(int count) {
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        min_x = min_y = min_z = 32767;
        max_x = max_y = max_z = -32767;
        for (int i = 0; i < count; i++) {
            auto [x, y, z] = readMagRaw();
            min_x = std::min(min_x, x);
            min_y = std::min(min_y, y);
            min_z = std::min(min_z, z);
            max_x = std::max(max_x, x);
            max_y = std::max(max_y, y);
            max_z = std::max(max_z, z);
            LOG << "x " << x << ", y " << y << ", z " << z;
            // 75Hz ODR in continious mode
            // means ~15ms wait per reading
            mcl::sleep_ms(15);
        }
        // Hard iron biases
        x_bias = (max_x + min_x) / 2;
        y_bias = (max_y + min_y) / 2;
        z_bias = (max_z + min_z) / 2;
        // Soft iron scale factors
        auto avg_delta_x = (max_x - min_x) / 2;
        auto avg_delta_y = (max_y - min_y) / 2;
        auto avg_delta_z = (max_z - min_z) / 2;
        auto avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;
        x_scale = avg_delta / avg_delta_x;
        y_scale = avg_delta / avg_delta_y;
        z_scale = avg_delta / avg_delta_z;
        LOG << "-------------------------";
        LOG << "x_bias " << x_bias;
        LOG << "y_bias " << y_bias;
        LOG << "z_bias " << z_bias;
        LOG << "x_scale " << x_scale;
        LOG << "y_scale " << y_scale;
        LOG << "z_scale " << z_scale;
        LOG << "-------------------------";
    }

    auto setDeclination(double d) {
        _decl = d * (M_PI / 180); //radians
    }
};

}

#endif