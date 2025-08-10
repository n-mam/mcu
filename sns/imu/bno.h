#ifndef BNO_H
#define BNO_H

#include <tuple>
#include <cstring>

#include <mcl/i2c.h>

using YPR = std::tuple<double, double, double>;
using QAT = std::tuple<double, double, double, double>;
using CST = std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>;

struct bno {

    bno(int sda, int scl, uint freq = 400*1000) {
        i2c::initialize(sda, scl, freq);
    }

    ~bno() {}

    virtual auto getQuaternion() -> QAT = 0;
    virtual auto getEulerAngles() -> YPR = 0;
    virtual auto getCalibrationStatus() -> CST = 0;
};

#endif