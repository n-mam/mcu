#ifndef BNO_H
#define BNO_H

#include <tuple>
#include <cstring>

#include <mcl/i2c.h>

using YPR = std::tuple<double, double, double>;
using QAT = std::tuple<double, double, double, double>;
using CST = std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>;

struct bno {

    static serial::i2c *_i2c;

    bno(serial::i2c& bus) {
        _i2c = &bus;
    }

    ~bno() {}

    virtual auto getQuaternion() -> QAT = 0;
    virtual auto getEulerAngles() -> YPR = 0;
    virtual auto getCalibrationStatus() -> CST = 0;
};

serial::i2c *bno::_i2c = nullptr;

#endif