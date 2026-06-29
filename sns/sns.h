#ifndef SNS_H
#define SNS_H

#include <memory>

#include <sns/imu/bno055.h>
#include <sns/imu/bno085.h>
#include <sns/imu/mpu6050.h>
#if defined (PICO)
#include <sns/TOF/vl53l0x.h>
#endif
#include <sns/mag/hmc5883l.h>
#include <sns/MS5837/MS5837.h>
#include <sns/motor/motor.h>

namespace sensor {

template<typename T, typename... Args>
inline auto create(Args&&... args) {
    auto s = std::make_unique<T>(std::forward<Args>(args)...);
    #if defined (PICO)
    if (std::is_same_v<T, vl53l0x>) {
        auto ss = reinterpret_cast<vl53l0x *>(s.get());
        if (ss) {
            ss->init();
            ss->setTimeout(500);
            ss->startContinuous();
        }
    } else
    #endif
    if (std::is_same_v<T, MS5837>) {

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

} //namespace sensor

inline void test_bno055() {
    #if defined (STM32)
    auto imu = sensor::create<imu::bno55>(7, 6, 400'000);
    #elif defined (PICO)
    auto imu = sensor::create<imu::bno55>(16, 17, 400'000);
    #endif
    while (true) {
        auto [gyro, accl, mag, sys] = imu->getCalibrationStatus();
        LOG << "{\"cal_gyro\":" << unsigned(gyro) << ", \"cal_acc\":" << unsigned(accl)
            << ", \"cal_mag\":" << unsigned(mag) << ", \"cal_sys\":" << unsigned(sys) << "}\n";
        // accl
        //bno055_accel_float_t accel;
        //bno055_convert_float_accel_xyz_msq(&accel);
        //LOG << std::fixed << std::setprecision(2);
        //LOG << "{\"acc_x\":" << accel.x << ", \"acc_y\":" << accel.y << ", \"acc_z\":" << accel.z << "}\n";
        auto [h, p, r] = imu->getEulerAngles();
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "{\"y\":" << h << ", \"p\":" << p << ", \"r\":" << r << "}\n";
        LOG << ss.str();
        mcl::sleep_ms(100);
    }
}

inline void test_bno085() {
    #if defined (STM32)
    auto imu = sensor::create<imu::bno85>(7, 6, 400'000);
    #elif defined (PICO)
    auto imu = sensor::create<imu::bno85>(16, 17, 400'000);
    #endif
    while (true) {
        auto [h, p, r] = imu->getEulerAngles();
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "{\"y\":" << h << ", \"p\":" << p << ", \"r\":" << r << "}\n";
        LOG << ss.str();
        mcl::sleep_ms(100);
    }
}

inline void test_hmc58883l() {
    imu::HMC5883L compass(20, 21) ;
    compass.setDeclination(4);
    //compass.calibrate(2000);
    while (true) {
        auto heading = compass.getHeading();
        printf("heading: %f\n", heading);
        mcl::sleep_ms(700);
    }
}

inline void test_ms5837() {
    #if defined (STM32)
    auto ms5837 = sensor::create<MS5837>(7, 6, 400'000);
    #elif defined (PICO)
    auto ms5837 = sensor::create<MS5837>(16, 17, 400'000);
    #endif
    ms5837->init();
    mcl::sleep_ms(5000);
    ms5837->setModel(MS5837::MS5837_30BA);
    // kg/m^3 (freshwater, 1029 for seawater)
    ms5837->setFluidDensity(997);
    while (true) {
        ms5837->read();
        LOG << "Pressure: " << ms5837->pressure() << " mbar";
        LOG << "Temperature: " << ms5837->temperature() << " deg C";
        LOG << "Depth: " << ms5837->depth() << " m";
        LOG << "Altitude: " << ms5837->altitude() << " m above mean sea level\n";
        mcl::sleep_ms(1000);
    }
}

inline void test_vl53l0x() {
    #if defined (PICO)
    int i = 0;
    std::array<uint64_t, 5> mm;
    auto tof = sensor::create<vl53l0x>(16, 17, 400'000);
    while (!getInstance<config>()->shouldExit()) {
        auto d = tof->readRangeContinuousMillimeters();
        if (!tof->timeoutOccurred()) {
            mm[i] = d - 60;
            LOG << " " << mm[i] << " mm";
            i = (i + 1) % mm.size();
        }
        mcl::sleep_ms(50);
    }
    #endif
}

inline void test_mpu6050() {
    #if defined (STM32)
    auto mpu = sensor::create<imu::mpu6050>(7, 6, 400'000);
    #elif defined (PICO)
    auto mpu = sensor::create<imu::mpu6050>(16, 17, 400'000);
    #endif
    mpu->initialize();
    mpu->calibrate();
    while (true) {
        auto [ax, ay, az, gx, gy, gz] = mpu->read_calibrated();
        LOG << "accl: " << ax << " " << ay << " " << az
                << "| gyro: " << gx << " " << gy << " " << gz;
    }
}

#endif