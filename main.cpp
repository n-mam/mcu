#include <sns/sns.h>

static constexpr
    std::pair<int, void(*)()>
        dispatch_table[] = {
            { 1,  mcl::test_led },
            { 2,  mcl::test_m200 },
            { 3,  mcl::test_blhS },
            { 4,  test_ms5837 },
            { 5,  test_bno055 },
            { 6,  test_bno085 },
            { 7,  mcl::test_serial },
            { 8,  test_hmc58883l },
            { 9,  mcl::test_sg90_servo },
            {10,  mcl::test_motor_drv8833 },
            {11,  mcl::toggle_default_led },
            {12,  test_network_nanopb },
            {13,  test_cdc_nanopb },
            {14,  test_vl53l0x },
            {15,  mcl::test_28BYJ_48_stepper },
            {16,  mcl::test_bldc_trapezoidal_ll },
            {17,  mcl::test_bldc_trapezoidal_pwm },
            {18,  mcl::test_bldc_trapezoidal_pwm_comp },
            {19,  mcl::test_bldc_sinusoidal_wave },
            {20,  test_mpu6050},
            #if defined(PICO)
            {99, []() {
                watchdog_enable(3000, true);
                while (true) {}
            }},
            #endif
};

int main(void) {
    mcl::initialize();
    auto *c = getInstance<config>();
    //while(!c->getKeyValue(config::key::action)) { mcl::send_discovery(); };
    while (true) {
        auto action = 20; //static_cast<int>(c->getKeyValue(config::key::action));
        LOG << "waiting for new action.. " << action;
        auto it = std::ranges::find_if(dispatch_table,
            [action](const auto& e){ return e.first == action; });
        if (it != std::end(dispatch_table)) {
            it->second();
        } else {
            LOG << "Unknown action: " << action;
        }
        mcl::sleep_ms(500);
    }
}

