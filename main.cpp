#include <sns/sns.h>
#include <mcl/motor.h>
#include <mcl/serial.h>

void test_led();
void test_m200();
void test_blhS();
void test_ms5837();
void test_bno055();
void test_bno085();
void test_serial();
void test_vl53l0x();
void test_hmc58883l();
void test_cdc_nanopb();
void test_sg90_servo();
void toggle_default_led();
void test_motor_drv8833();
void test_network_nanopb();
void test_28BYJ_48_stepper();
void test_bldc_trapezoidal_ll();
void test_bldc_sinusoidal_wave();
void test_bldc_trapezoidal_pwm();

config *c = nullptr;

int main(void) {
    mcl::initialize();
    auto c = getInstance<config>();
    while(!c->getValue(config::key::action)) { mcl::send_discovery(); };
    while (true) {
        auto action = (int)c->getValue(config::key::action);
        LOG << "waiting for new action.. " << action;
        if (action == 1) {
            test_led();
        } else if (action == 2) {
            test_m200();
        } else if (action == 3) {
            test_blhS();
        } else if (action == 4) {
            test_ms5837();
        } else if (action == 5) {
            test_bno055();
        } else if (action == 6) {
            test_bno085();
        } else if (action == 7) {
            test_serial();
        } else if (action == 8) {
            test_hmc58883l();
        } else if (action == 9) {
            test_sg90_servo();
        } else if (action == 10) {
            test_motor_drv8833();
        } else if (action == 11) {
            toggle_default_led();
        } else if (action == 12) {
            test_network_nanopb();
        } else if (action == 13) {
            test_cdc_nanopb();
        } else if (action == 14) {
            test_vl53l0x();
        } else if (action == 15) {
            test_28BYJ_48_stepper();
        } else if (action == 16) {
            test_bldc_trapezoidal_ll();
        } else if (action == 17) {
            test_bldc_trapezoidal_pwm();
        } else if (action == 18) {
            test_bldc_sinusoidal_wave();
        } else if (action == 99) {
            #if defined (PICO)
            watchdog_enable(3000, true);
            while(true) {};
            #endif
        }
        mcl::sleep_ms(500);
    }
}

#if defined (STM32)
void test_bldc_sinusoidal_wave() {
    // on nucleo-64 boards PA2 PA3
    // are used by stlink usart
    // use PB timer AF instead
    #if defined (STM32)
    Timer t2(TIM2);
    t2.set_frequency(50);
    // TIM2 CH1 PB0
    t2.init_channel(1, GPIOB, 8);
    t2.set_duty_cycle(1, 0);
    t2.start_channel(1);
    // TIM2 CH2 PB1
    t2.init_channel(2, GPIOB, 9);
    t2.set_duty_cycle(2, 0);
    t2.start_channel(2);
    // TIM2 CH3 PB3
    t2.init_channel(3, GPIOB, 10);
    t2.set_duty_cycle(3, 0);
    t2.start_channel(3);
    // Enable the timer
    t2.enable();
    #endif
    while (true){mcl::sleep_ms(100);}
}

// phases A, B and C same as
// phases U, V and W phases

typedef struct {
    uint8_t HA, LA;
    uint8_t HB, LB;
    uint8_t HC, LC;
} Step;

Step comm_table[6] = {
    {1, 0, 0, 1, 0, 0},  // Step 1: A+ B-
    {1, 0, 0, 0, 0, 1},  // Step 2: A+ C-
    {0, 0, 1, 0, 0, 1},  // Step 3: B+ C-
    {0, 1, 1, 0, 0, 0},  // Step 4: B+ A-
    {0, 1, 0, 0, 1, 0},  // Step 5: C+ A-
    {0, 0, 0, 1, 1, 0},  // Step 6: C+ B-
};

void apply_step_pwm(Step s, double duty, Timer& timer) {
    GPIOA->BSRR =
        (s.LA ? GPIO_BSRR_BS1 : GPIO_BSRR_BR1) |
        (s.LB ? GPIO_BSRR_BS3 : GPIO_BSRR_BR3) |
        (s.LC ? GPIO_BSRR_BS5 : GPIO_BSRR_BR5);
    timer.set_duty_cycle(1, duty);
    timer.set_duty_cycle(2, duty);
    timer.set_duty_cycle(3, duty);
}

void apply_step_ll(Step s) {
    GPIOA->BSRR =
        (s.HA ? GPIO_BSRR_BS0 : GPIO_BSRR_BR0) |
        (s.LA ? GPIO_BSRR_BS1 : GPIO_BSRR_BR1) |
        (s.HB ? GPIO_BSRR_BS2 : GPIO_BSRR_BR2) |
        (s.LB ? GPIO_BSRR_BS3 : GPIO_BSRR_BR3) |
        (s.HC ? GPIO_BSRR_BS4 : GPIO_BSRR_BR4) |
        (s.LC ? GPIO_BSRR_BS5 : GPIO_BSRR_BR5);
}

void test_bldc_trapezoidal_ll() {
    // Enable clock for the GPIOA
    mcl::enableClockForGpio(GPIOA);
    // Clear mode bits for PA0 to PA5 first
    GPIOA->MODER &= ~(
        (3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) |
        (3 << (3 * 2)) | (3 << (4 * 2)) | (3 << (5 * 2)));
    // Set PA0 to PA5 as general purpose output
    GPIOA->MODER |= (
        (1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2)) |
        (1 << (3 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2)));
    // Optional: Set as push-pull, low-speed (default)
    uint8_t step = 0;
    while (true) {
        // All off
        apply_step_ll({0, 0, 0, 0, 0, 0});
        // Dead time
        mcl::sleep_ms(5);
        // Next commutation step
        apply_step_ll(comm_table[step]);
        // Hold for the motor to react
        mcl::sleep_ms(100);
        step = (step + 1) % 6;
    }
}

void test_bldc_trapezoidal_pwm() {
    #if defined (STM32)
    // Enable clock
    mcl::enableClockForGpio(GPIOA);
    // // Clear mode bits for PA1, PA3, PA5 as GPIO outputs (LA, LB, LC)
    GPIOA->MODER &= ~((3 << (1 * 2)) | (3 << (3 * 2)) | (3 << (5 * 2)));
    // Set PA1, PA3, PA5 as GP outputs (LA, LB, LC)
    GPIOA->MODER |=  ((1 << (1 * 2)) | (1 << (3 * 2)) | (1 << (5 * 2)));
    // Configure PA8, PA9, PA10 as HS PWM
    Timer tm(TIM4);
    // 20KHz BLDC frequency
    tm.set_frequency(20*1000);
    // TIM4 CH1 PB6
    tm.init_channel(1, GPIOB, 6);
    tm.set_duty_cycle(1, 0);
    tm.start_channel(1);
    // TIM4 CH2 PB7
    tm.init_channel(2, GPIOB, 7);
    tm.set_duty_cycle(2, 0);
    tm.start_channel(2);
    // TIM4 CH3 PB8
    tm.init_channel(3, GPIOB, 8);
    tm.set_duty_cycle(3, 0);
    tm.start_channel(3);
    // Enable the timer
    tm.enable();
    #endif
    int step = 0;
    double duty = 0.30;
    while (!getInstance<config>()->shouldExit()) {
        // All off
        apply_step_pwm({0,0,0,0,0,0}, 0, tm);
        // Dead time
        mcl::sleep_ms(5);
        // Next commutation step
        duty = getInstance<config>()->getValue(config::key::motor);
        apply_step_pwm(comm_table[step], duty, tm);
        // Hold for the motor to react
        mcl::sleep_ms(10);
        step = (step + 1) % 6;
    }
    apply_step_pwm({0,0,0,0,0,0}, 0, tm);
}
#elif defined (PICO)

void test_bldc_trapezoidal_ll() {}
void test_bldc_sinusoidal_wave() {}
void test_bldc_trapezoidal_pwm() {}

#endif

void test_vl53l0x() {
    #if defined (PICO)
    int i = 0;
    std::array<uint64_t, 5> mm;
    auto tof = sensor::create<vl53l0x>(16, 17, 400*1000);
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

void test_serial() {
    #if defined (STM32F4)
    // Create USART instance with TX on PA2, RX on PA3
    mcl::serial serial(2, 3, 115200, 8, false);
    const char *message = "Hello, USART! This is a message from blackpill\n";
    #elif defined (STM32F7)
    // Create USART instance with TX on PG14, RX on PG9
    mcl::serial serial(14, 9, 115200, 8, false);
    const char *message = "Hello, USART! This is a message from F7\n";
    #elif defined (STM32H7)
    // todo: copied from f7
    // Create USART instance with TX on PG14, RX on PG9
    mcl::serial serial(14, 9, 115200, 8, false);
    const char *message = "Hello, USART! This is a message from F7\n";
    #elif defined (PICO)
    // todo
    mcl::serial serial(1, 2, 115200, 8, false);
    const char *message = "UASRT to be implemented for pico\n";
    #endif
    while (1) {
        serial.transmit(reinterpret_cast<const uint8_t*>(message), strlen(message));
        mcl::sleep_ms(1000);
    }
}

void test_led() {
    mcl::pwm _pwm(5, PWM_FREQ_LED);
    _pwm.start(100);
    while (true) {
        for (uint16_t i = 0; i < 100; i++) {
            _pwm.set_duty_cycle(0.01 * i);
            mcl::sleep_ms(50);
        }
        for (uint16_t i = 100; i > 0; i--) {
            _pwm.set_duty_cycle(0.01 * i);
            mcl::sleep_ms(50);
        }
    }
}

void test_network_nanopb() {
    #if defined(PICO_CYW43_SUPPORTED)
    bool flag = false;
    uint32_t counter = 1;
    while (true) {
        nanomsg::write_nano_msg_a("hello from pico !", mcl::log::sink::net);
        nanomsg::write_nano_msg_b(counter++, mcl::log::sink::net);
        nanomsg::write_nano_msg_c(flag = !flag, mcl::log::sink::net);
        mcl::sleep_ms(1000);
    }
    #endif
}

void test_cdc_nanopb() {
    #if defined (STM32)
    bool flag = false;
    uint32_t counter = 1;
    while (true) {
        nanomsg::write_nano_msg_a("hello from stm32 !", mcl::log::sink::cdc);
        nanomsg::write_nano_msg_b(counter++, mcl::log::sink::cdc);
        nanomsg::write_nano_msg_c(flag = !flag, mcl::log::sink::cdc);
        mcl::sleep_ms(1000);
    }
    #endif
}

void toggle_default_led() {
    #if defined (STM32F411xE)
    // STM32F411CEU6 black pill
    // Enable AHB1 clock to GPIO C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    // Set GPIOC's pin 13 mode to GP output
    GPIOC->MODER |= GPIO_MODER_MODER13_0;
    // Set GPIOC's pin 13 to "very high" speed output
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED13;
    while (true) {
        std::cout <<"toggle PC13" << std::endl;
        GPIOC->ODR ^= (1 << 13);
        // systick 1ms timer test
        mcl::sleep_ms(500);
    }
    #elif defined (STM32F446xx)
    // Enable AHB1 clock to GPIO A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set GPIOA's pin 5 mode to GP output
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
    // Set GPIOA's pin 5 to "very high" speed output
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5;
    while (true) {
        std::cout <<"toggle PA5" << std::endl;
        GPIOA->ODR ^= (1 << 5);
        // systick 1ms timer test
        mcl::sleep_ms(500);
    }
    #elif defined (STM32F7)
    // NUCLEO-F767ZI STM32F767ZI
    // User LD2: a blue user LED is connected to PB7
    // Enable AHB1 clock to GPIO C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // Set GPIOB's pin 7 mode to GP output
    GPIOB->MODER |= GPIO_MODER_MODER7_0;
    // Set GPIOB's pin 7 to "very high" speed output
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR7;
    while (true) {
        std::cout <<"toggle PB7" << std::endl;
        GPIOB->ODR ^= (1 << 7);
        // systick 1ms timer test
        mcl::sleep_ms(500);
    }
    #elif defined (PICO)
        #if defined(PICO_DEFAULT_LED_PIN)
            // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
            // so we can use normal GPIO functionality to turn the led on and off
            gpio_init(PICO_DEFAULT_LED_PIN);
            gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        #elif defined(CYW43_WL_GPIO_LED_PIN)
            // Unlike Pico, the on-board LED on Pico W is not connected to a
            // pin on RP2040, but instead to a GPIO pin on the wireless chip
        #endif
        while (true) {
            #if defined(PICO_DEFAULT_LED_PIN)
                // Just set the GPIO on or off
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
            #elif defined(CYW43_WL_GPIO_LED_PIN)
                // Ask the wifi "driver" to set the GPIO on or off
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            #endif
            mcl::sleep_ms(500);
            #if defined(PICO_DEFAULT_LED_PIN)
                // Just set the GPIO on or off
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
            #elif defined(CYW43_WL_GPIO_LED_PIN)
                // Ask the wifi "driver" to set the GPIO on or off
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            #endif
            mcl::sleep_ms(500);
        }
    #endif
}

void test_bno085() {
    #if defined (STM32F4) || defined (STM32F7) || defined (STM32H7)
    auto imu = sensor::create<imu::bno85>(7, 6, 400*1000);
    #elif defined (PICO)
    auto imu = sensor::create<imu::bno85>(16, 17, 400*1000);
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

void test_bno055() {
    #if defined (STM32F4) || defined (STM32F7) || defined (STM32H7)
    auto imu = sensor::create<imu::bno55>(7, 6, 400*1000);
    #elif defined (PICO)
    auto imu = sensor::create<imu::bno55>(16, 17, 400*1000);
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

void test_hmc58883l() {
    imu::HMC5883L compass(20, 21) ;
    compass.setDeclination(4);
    //compass.calibrate(2000);
    while (true) {
        auto heading = compass.getHeading();
        printf("heading: %f\n", heading);
        mcl::sleep_ms(700);
    }
}

void test_ms5837() {
    #if defined (STM32F4) || defined (STM32F7) || defined (STM32H7)
    auto ms5837 = sensor::create<MS5837>(7, 6, 400*1000);
    #elif defined (PICO)
    auto ms5837 = sensor::create<MS5837>(16, 17, 400*1000);
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

inline void test_m200() {
    auto m200 = mcl::initialize_m200(PWM_PIN_MOTOR, PWM_FREQ_MOTOR);
    while (!getInstance<config>()->shouldExit()) {
        LOG << "looping...1";
        m200->set_duty_cycle(getInstance<config>()->getValue(config::key::motor) / 20.0);
        mcl::sleep_ms(1000);
    }
}

inline void test_blhS() {
    auto blhS = mcl::initialize_blheliS(PWM_PIN_MOTOR, PWM_FREQ_MOTOR, false);
    while (true) {
        LOG << "blhS looping... 1.1";
        blhS->set_duty_cycle(1.1 / 20);
        mcl::sleep_ms(3000);
        LOG << "blhS looping ... 1.2";
        blhS->set_duty_cycle(1.2 / 20);
        mcl::sleep_ms(3000);
        LOG << "blhS looping ... 1.3";
        blhS->set_duty_cycle(1.3 / 20);
        mcl::sleep_ms(3000);
    }
}

inline void test_servo_m200() {
    auto m200 = mcl::initialize_m200(PWM_PIN_MOTOR, PWM_FREQ_MOTOR);
    auto rudder = mcl::initialize_rudder(PWM_PIN_RUD_SERVO, PWM_FREQ_SERVO);
    while (true) {
        m200->set_duty_cycle(1.6 / 20.0);
        rudder->set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(500);
        m200->set_duty_cycle(1.6 / 20.0);
        rudder->set_duty_cycle(1.5 / 20.0);
        mcl::sleep_ms(500);
        m200->set_duty_cycle(1.6 / 20.0);
        rudder->set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(500);
    }
    m200->set_duty_cycle(1.5 / 20.0);
}

inline void test_servo_drv8833_motor() {
    mcl::pwm _pwm_servo(0, PWM_FREQ_SERVO);
    mcl::motor m(14, 15, PWM_FREQ_MOTOR_DRV8833);
    printf("starting servo and motor...\n");
    m.start();
    _pwm_servo.start();
    mcl::sleep_ms(5000);
    while (true) {
        // MG990 0-180 degrees
        m.set_speed(95);
        _pwm_servo.set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(1000);
        m.set_speed(90);
        _pwm_servo.set_duty_cycle(1.5 / 20.0);
        mcl::sleep_ms(1000);
        m.set_speed(85);
        _pwm_servo.set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(1000);
        m.set_speed(80);
        _pwm_servo.set_duty_cycle(0.5 / 20.0);
        mcl::sleep_ms(1000);
        _pwm_servo.set_duty_cycle(2.5 / 20.0);
        mcl::sleep_ms(1000);
    }
}

inline void test_sg90_servo() {
    // for servos the pwm frequency is ideally in a range of 40-200Hz. 50Hz frequency
    // implies a cycle(pulse) every 20ms. The servo angle is determined by the pulse width in a 50 Hz PWM signal.
    // Most servos move to 0 when they receive a pulse 1500 µs long. Generally it is safe to send a servo a pulse in the range 1000 µs to 2000 µs.
    // Generally a 10 µs change in pulse width results in a 1 degree change in angle. At some point you will reach the limit of rotation. That limit
    // varies between different makes and models of servos. If you try to force a servo beyond its limits it will get very hot (possibly to destruction)
    // and may strip its gears. The small 9g servos generally have an extended angle range, 180 degrees or more. Typically they accept pulse widths in the
    // range 500 µs to 2500 µs. Determine a servos limits carefully by experiment.
    mcl::pwm _pwm(0, PWM_FREQ_SERVO);
    mcl::sleep_ms(5000);
    printf("starting servo...\n");
    _pwm.start();
    while (!getInstance<config>()->shouldExit()) {
        LOG << "test servo looping...5";
        // _pwm.set_duty_cycle(2.6 / 20.0);
        // mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(2.4 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(2.2 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(2.0 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.8 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.6 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.5 / 20.0); // center
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.4 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.2 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(1.0 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(0.8 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(0.6 / 20.0);
        mcl::sleep_ms(1000);
        _pwm.set_duty_cycle(0.4 / 20.0);
        mcl::sleep_ms(1000);
    }
}

inline void test_motor_drv8833() {
    mcl::motor motor(14, 15, 20000);
    motor.start();
    // brushed motor might have a differnt
    // starting duty cycle in reverse direction
    mcl::sleep_ms(1000);
    while (1) {
        motor.set_speed(95);
    }
    // mcl::sleep_ms(1500);
    // motor.set_speed(90);
    // mcl::sleep_ms(1500);
    // motor.set_speed(85);
    // mcl::sleep_ms(1500);
    // motor.set_speed(80);
    // mcl::sleep_ms(1500);
    // motor.set_speed(75);
    // mcl::sleep_ms(1500);
    // motor.set_speed(70);
    // mcl::sleep_ms(1500);
    // motor.set_speed(65);
    // mcl::sleep_ms(1500);
    // motor.set_speed(60);
    // mcl::sleep_ms(1500);
    // motor.set_speed(55);
    // mcl::sleep_ms(1500);
    // motor.set_speed(50);
    // mcl::sleep_ms(1500);
    // motor.set_speed(45);
    // mcl::sleep_ms(3000);
    // motor.reverse();
    // mcl::sleep_ms(1500);
    // motor.set_speed(50);
}

void test_28BYJ_48_stepper() {
    #if defined PICO
    const int in[4] = {2, 3, 4, 5};
    std::array<std::array<uint, 4>, 8>
        sequence = {{
            {1, 0, 0, 0},
            {1, 1, 0, 0},
            {0, 1, 0, 0},
            {0, 1, 1, 0},
            {0, 0, 1, 0},
            {0, 0, 1, 1},
            {0, 0, 0, 1},
            {1, 0, 0, 1}
        }};
    for (auto& e : in) {
        gpio_init(e);
        gpio_set_dir(e, GPIO_OUT);
    }
    while (true) {
        for (const auto& step : sequence) {
            for (auto i = 0; i < 4; i++) {
                gpio_put(in[i], step[i]);
            }
            sleep_ms(10);
        }
        // for (const auto& step : stepSequence | std::views::reverse) {
        //     for (auto i = 0; i < 4; i++) {
        //         gpio_put(in[i], step[i]);
        //     }
        //     sleep_ms(10);
        // }
    }
    #endif
}
