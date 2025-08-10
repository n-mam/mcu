#ifndef SNS_H
#define SNS_H

#include <memory>

#include <sns/imu/bno055.h>
#include <sns/imu/bno085.h>
//#include <sns/TOF/vl53l0x.h>
#include <sns/mag/hmc5883l.h>
#include <sns/MS5837/MS5837.h>

namespace sensor {

template<typename T, typename... Args>
inline auto create(Args&&... args) {
    auto s = std::make_unique<T>(std::forward<Args>(args)...);
    /*if (std::is_same_v<T, vl53l0x>) {
        auto ss = reinterpret_cast<vl53l0x *>(s.get());
        if (ss) {
            ss->init();
            ss->setTimeout(500);
            ss->startContinuous();
        }
    } else*/ if (std::is_same_v<T, MS5837>) {

    } else if (std::is_same_v<T, imu::bno55>) {

    } else if (std::is_same_v<T, imu::bno85>) {
        auto ss = reinterpret_cast<imu::bno85 *>(s.get());
        auto rc = ss->init_i2c_hal();
        if (!rc) {
            LOG << "init_i2c_hal failed";
            return s;
        }
        rc = ss->enableReports();
        if (!rc) {
            LOG << "enableReports failed";
            return s;
        }
        rc = ss->enableCalibration();
        if (!rc) {
            LOG << "enableCalibration failed";
            return s;
        }
    }
    mcl::sleep_ms(500);
    return s;
}

}

#endif