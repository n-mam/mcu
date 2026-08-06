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
    #if defined (PICO)
    serial::i2c bus(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus(7, 6, 400'000, I2C1, GPIOB);
    #endif
    auto imu = sensor::create<imu::bno55>(bus);
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
    #if defined (PICO)
    serial::i2c bus(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus(7, 6, 400'000, I2C1, GPIOB);
    #endif
    auto imu = sensor::create<imu::bno85>(bus);
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
    #if defined (PICO)
    serial::i2c bus(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus(7, 6, 400'000, I2C1, GPIOB);
    #endif
    auto compass = sensor::create<imu::hmc5883l>(bus);
    compass->setDeclination(4);
    compass->calibrate(2000);
    while (true) {
        auto heading = compass->getHeading();
        LOG << "heading: " << heading;
        mcl::sleep_ms(700);
    }
}

inline void test_ms5837() {
    #if defined (PICO)
    serial::i2c bus(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus(7, 6, 400'000, I2C1, GPIOB);
    #endif
    auto ms5837 = sensor::create<MS5837>(bus);
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
    #if defined (PICO)
    serial::i2c bus(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus(7, 6, 400'000, I2C1, GPIOB);
    #endif
    auto mpu = sensor::create<imu::mpu6050>(bus);
    mpu->initialize();
    mpu->calibrate();
    while (true) {
        auto [ts, ax, ay, az, gx, gy, gz] = mpu->read_calibrated();
        LOG << " " << ts << "," << ax << "," << ay << "," << az << ","
                << gx << "," << gy << "," << gz;
    }
}

inline void test_mahony() {
    // MPU6050
    #if defined (PICO)
    serial::i2c bus1(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus1(7, 6, 400'000, I2C1, GPIOB);
    #endif
    auto mpu = sensor::create<imu::mpu6050>(bus1);
    mpu->initialize();
    mpu->calibrate();
    // hmc5883l
    #if defined (PICO)
    serial::i2c bus2(16, 17, 400'000, i2c0);
    #elif defined (STM32)
    serial::i2c bus2(3, 10, 400'000, I2C2, GPIOB); //F466
    #endif
    auto mag = sensor::create<imu::hmc5883l>(bus2);
    mag->setDeclination(4);
    mag->calibrate(2000);
    while (true) {
        //double gx, gy, gz; gx = gy = gz = 0.0;
        //double ax, ay, az; ax = ay = az = 0.0;
        //double mx, my, mz; mx = my = mz = 0.0;
        auto [mx, my, mz] = mag->readMagnetometer();
        auto [ts, ax, ay, az, gx, gy, gz] = mpu->read_calibrated();
        LOG << " " << ts << ","
            << ax << "," << ay << "," << az << ","
                << gx << "," << gy << "," << gz << ","
                    << mx << "," << my << "," << mz;
        mcl::sleep_ms(10);
    }
}

// TIM2 update event →
//   ADC1 external trigger →
//      ADC conversion →
//         DMA2 Stream0 →
//            circular buffer
extern "C" {
    volatile uint32_t dma_half_count = 0;
    volatile uint32_t dma_full_count = 0;
    volatile uint32_t dma_err_count = 0;
    volatile uint32_t adc_ovr_count = 0;
    alignas(4) uint16_t adc_buffer[1024];
    alignas(4) uint16_t print_buffer[512];
    volatile bool half_ready = false;
    volatile bool full_ready = false;
    inline void adc_callback(int n) {
        if (n == 0) {
            adc_ovr_count++;
        }
    }
    inline void dma_callback(int n) {
        if (n == 1) {
            dma_half_count++;
            half_ready = true;
        } else if (n == 2) {
            dma_full_count++;
            full_ready = true;
        } else if (n == 3) {
            dma_err_count++;
        }
    }
}

inline void adc_tim_dma_test() {
    #if defined(STM32F4) || defined(STM32F7)
    // Enable clock for GPIOA (sampling pin)
    mcl::enableClockForGpio(GPIOA);
    // PA0 -> Analog mode
    GPIOA->MODER &= ~(3UL << (0 * 2));
    GPIOA->MODER |=  (3UL << (0 * 2));
    // No pull-up/pull-down
    GPIOA->PUPDR &= ~(3UL << (0 * 2));
    // ADC configuration
    ADC_Config_t adc = {
        ADC1,                 // instance
        ADC_CH0,              // channel
        ADC_ALIGN_RIGHT,      // alignment
        ADC_RES_12BIT,        // resolution
        ADC_SAMPLE_84,        // sampleTime
        ADC_TRIGGER_TIM2_TRGO, // EXTSEL (TIM2)
        adc_callback           // interrupt handler
    };
    // Enable clock for ADC peripheral
    // to access ADC common registers
    mcl::enableClockForAdc(ADC1);
    adc_global_init();
    adc_init(&adc);
    // DMA configurtion
    DMA_Config_t dma = {
        0,                          // channel
        DMA_SxCR_MSIZE_0,           // memorySize (16-bit)
        true,                       // circularMode
        DMA2,                       // instance
        1024,                       // transferCount
        true,                       // memoryIncrement
        DMA_SxCR_PSIZE_0,           // peripheralSize (16-bit)
        DMA_DIR_PER_TO_MEM,         // direction
        DMA2_Stream0,               // stream
        false,                      // peripheralIncrement
        adc_buffer,                 // memoryAddress
        &ADC1->DR,                  // peripheralAddress
        dma_callback                // interrupt handler
    };
    dma_init(&dma);
    dma_enable_irq(&dma);
    // Timer config
    timer_config_t tim{};
    tim.instance = TIM2;
    timer_init(&tim);
    timer_set_frequency(&tim, 1000);
    timer_enable_trgo(&tim);
    // start DMA
    dma_start(&dma);
    // enable ADC
    adc_enable(&adc);
    // start timer trigger source last
    timer_start(&tim);
    while(true) {
        // LOG << " half=" << dma_half_count
        //     << " full=" << dma_full_count
        //     << " d_err=" << dma_err_count
        //     << " a_err=" << adc_ovr_count
        //     << " ADC_SR=" << (uint32_t)ADC1->SR
        //     << " TIM_CNT=" << (uint32_t)TIM2->CNT
        //     << " ADC_CR2=" << (uint32_t)ADC1->CR2
        //     << " DMA_LISR=" << (uint32_t)DMA2->LISR
        //     << " DMA_CR=" << (uint32_t)DMA2_Stream0->CR
        //     << " DMA_FCR=" << (uint32_t)DMA2_Stream0->FCR
        //     << " DMA_NDTR=" << (uint32_t)DMA2_Stream0->NDTR;
        if (half_ready) {
            half_ready = false;
            // copy first half
            memcpy(print_buffer, &adc_buffer[0], sizeof(print_buffer));
            for (size_t i = 0; i < 512; ++i) {
                LOG << " " << print_buffer[i];
            }
        }
        if (full_ready) {
            full_ready = false;
            // copy second half
            memcpy(print_buffer, &adc_buffer[512], sizeof(print_buffer));
            for (size_t i = 0; i < 512; ++i) {
                LOG << " " << print_buffer[i];
            }
        }
    }
    #endif
}

#endif