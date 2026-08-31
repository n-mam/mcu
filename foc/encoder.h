#ifndef ENCODER_H
#define ENCODER_H

// DFRobot 2804, 12N/14P
constexpr uint32_t POLE_PAIRS = 7;
constexpr float AS5600_COUNTS = 4096.0f;
constexpr float AS5600_COUNT_TO_RAD = TWO_PI / AS5600_COUNTS;

struct encoder_state_t {
    // Calibration: we want to determine two things
    // Zero — where is the encoder when the motor is at electrical angle 0?
    // Sign — when we command positive electrical rotation, does the encoder count go up or down?
    // +1: encoder count increases with positive electrical rotation
    // -1: encoder count decreases with positive electrical rotation
    int8_t sign = 1;
    // AS5600 raw count corresponding to electrical zero
    uint16_t zero_raw = 0;
    // Latest raw AS5600 reading
    uint16_t raw = 0;
    uint16_t previous_raw = 0;
    // Position states
    // Mechanical rotor angle in radians
    // Range: approximately [-PI, PI]
    float mechanical_angle = 0.0f;
    // Electrical angle for FOC Park transform
    // Range: [0, TWO_PI)
    float electrical_angle = 0.0f;
    // Velocity states
    // Mechanical angular velocity in rad/s
    float mechanical_velocity = 0.0f;
    // Electrical angular velocity in rad/s
    // = mechanical_velocity * POLE_PAIRS
    float electrical_velocity = 0.0f;
    // Velocity estimator state
    uint32_t last_update_cycles = 0;
    float velocity_last_angle = 0.0f;
    bool velocity_valid = false;
};

struct velocity_estimator_t {
    float filtered_velocity = 0.0f;
    float time_constant_s = 0.005f;
    bool initialized = false;
    // Trace-only.
    float raw_velocity = 0.0f;
};

inline encoder_state_t encoder_a;
inline encoder_state_t encoder_b;
inline encoder_state_t* encoder_write = &encoder_a;
inline encoder_state_t* encoder_read  = &encoder_b;
inline velocity_estimator_t g_vel_est;

inline void reset_velocity_estimator() {
    g_vel_est = {};
    encoder_a.mechanical_velocity = 0.0f;
    encoder_a.electrical_velocity = 0.0f;
    encoder_a.previous_raw = encoder_a.raw;
    encoder_b.mechanical_velocity = 0.0f;
    encoder_b.electrical_velocity = 0.0f;
    encoder_b.previous_raw = encoder_b.raw;
    const uint32_t now = DWT->CYCCNT;
    encoder_a.last_update_cycles = now;
    encoder_b.last_update_cycles = now;
}

inline uint16_t as5600_read_raw_angle(serial::i2c& bus) {
    // 2-byte reads on F446
    // should not be interruptible
    uint8_t buf[3] = {0};
    //__disable_irq();
    bus.read(0x36, 0x0C, buf, 3);
    //__enable_irq();
    return ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
}

inline uint16_t average_encoder_raw(serial::i2c& bus) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < 100; ++i) {
        sum += as5600_read_raw_angle(bus);
        mcl::sleep_ms(1);
    }
    return (uint16_t)(sum / 100);
}

// calculate the shortest signed difference between two
// AS5600 raw angles, accounting for the 0/4095 wraparound.
inline int32_t encoder_signed_wrap_delta(uint16_t from, uint16_t to) {
    // AS5600 is 12-bit: 0..4095.
    // Return shortest signed difference in [-2048, +2047].
    int32_t d = (int32_t)to - (int32_t)from;
    if (d > 2048)
        d -= 4096;
    else if (d < -2048)
        d += 4096;
    return d;
}

inline float encoder_to_mechanical_angle(
    uint16_t raw, uint16_t zero_raw, int8_t encoder_sign) {
    constexpr float AS5600_COUNTS = 4096.0f;
    float angle =
        ((float)raw - (float)zero_raw) / AS5600_COUNTS;
    // wrap to [-0.5,0.5] mechanical turns
    if (angle > 0.5f)
        angle -= 1.0f;
    else if (angle < -0.5f)
        angle += 1.0f;
    angle *= (float)encoder_sign;
    return angle * TWO_PI;
}

// Plain per-tick finite difference + first-order low-pass, velocity
// estimator. the longer filter tau does the noise/lag tradeoff work
inline void fd_velocity_update(int32_t delta_counts, float dt_s) {

    if (!g_vel_est.initialized || dt_s <= 1e-6f) {
        g_vel_est.initialized = true;
        g_vel_est.raw_velocity = 0.0f;
        return;
    }

    const float angle_delta = (float)delta_counts * AS5600_COUNT_TO_RAD;
    g_vel_est.raw_velocity = angle_delta / dt_s;

    const float alpha = dt_s / (g_vel_est.time_constant_s + dt_s);
    g_vel_est.filtered_velocity +=
        alpha * (g_vel_est.raw_velocity - g_vel_est.filtered_velocity);

    encoder_write->mechanical_velocity = g_vel_est.filtered_velocity;
    encoder_write->electrical_velocity =
        g_vel_est.filtered_velocity * (float)POLE_PAIRS;
}

// Extrapolate encoder state forward from the last I2C sample using
// the last measured velocity/acceleration. Called from the ADC ISR,
// which runs far more often than the encoder is actually read.
inline float encoder_predict_electrical_angle(const encoder_state_t& encoder_state) {

    const float encoder_age =
        (float)(DWT->CYCCNT - encoder_state.last_update_cycles) /
            (float)SystemCoreClock;

    float electrical_angle =
        encoder_state.electrical_angle +
        encoder_state.electrical_velocity * encoder_age;

    electrical_angle = fmodf(electrical_angle, TWO_PI);

    if (electrical_angle < 0.0f) {
        electrical_angle += TWO_PI;
    }

    return electrical_angle;
}

inline void encoder_read_and_update_angles(serial::i2c& bus) {

    // Start from the last published snapshot so the write buffer
    // contains a complete, coherent state before modifying it.
    *encoder_write = *encoder_read;

    encoder_write->raw = as5600_read_raw_angle(bus);

    encoder_write->mechanical_angle =
        encoder_to_mechanical_angle(
            encoder_write->raw,
            encoder_write->zero_raw,
            encoder_write->sign);

    float electrical_angle =
        encoder_write->mechanical_angle * (float)POLE_PAIRS;

    electrical_angle = fmodf(electrical_angle, TWO_PI);

    if (electrical_angle < 0.0f)
        electrical_angle += TWO_PI;

    encoder_write->electrical_angle = electrical_angle;

    // Velocity estimation
    const int32_t delta_counts =
        encoder_signed_wrap_delta(
            encoder_write->previous_raw,
            encoder_write->raw) *
        (int32_t)encoder_write->sign;

    const uint32_t now_cycles = DWT->CYCCNT;

    const float dt_s =
        (float)(now_cycles - encoder_write->last_update_cycles) /
            (float)SystemCoreClock;

    fd_velocity_update(delta_counts, dt_s);

    // trace_push(encoder_write->raw, delta_counts, dt_s,
    //            g_vel_est.raw_velocity, g_vel_est.filtered_velocity,
    //            encoder_write->electrical_angle);

    encoder_write->previous_raw = encoder_write->raw;
    encoder_write->last_update_cycles = now_cycles;

    // Publish the completely updated encoder state.
    encoder_state_t* tmp = encoder_read;
    encoder_read = encoder_write;
    encoder_write = tmp;
}

inline bool calibrate_encoder(serial::i2c& bus, svpwm_t& svm, float vbus) {
    constexpr float CALIBRATION_VOLTAGE = 1.5f;
    // Keep the modulation relatively small. It should be large enough to
    // overcome cogging/friction, but not so large that the rotor
    // heats unnecessarily.
    // Force the rotor to electrical angle 0.
    // Valpha = +V
    // Vbeta  =  0
    voltage_to_timer_pwm(svm.timer, CALIBRATION_VOLTAGE / vbus, 0.0f);
    mcl::sleep_ms(1000);
    encoder_write->zero_raw = average_encoder_raw(bus);
    LOG << " zero raw = " << encoder_write->zero_raw;
    // Move the electrical field +90 degrees.
    // Valpha = 0
    // Vbeta  = +V
    // If positive electrical rotation makes the encoder count
    // increase, sign = +1.
    // If it makes the encoder count decrease, sign = -1.
    voltage_to_timer_pwm(svm.timer, 0.0f, CALIBRATION_VOLTAGE / vbus);
    mcl::sleep_ms(1000);
    const uint16_t plus_90_raw = average_encoder_raw(bus);
    const int32_t delta = encoder_signed_wrap_delta(encoder_write->zero_raw, plus_90_raw);
    // Determine direction.
    // We expect a meaningful encoder movement. If there is almost
    // no movement, calibration should fail rather than silently
    // choosing a direction.
    constexpr int32_t MIN_DIRECTION_COUNTS = 5;
    if (delta > MIN_DIRECTION_COUNTS) {
        encoder_write->sign = +1;
    } else if (delta < -MIN_DIRECTION_COUNTS) {
        encoder_write->sign = -1;
    } else {
        encoder_write->sign = 0;
        return false;
    }
    LOG << " +90 raw = " << plus_90_raw;
    LOG << " delta = " << delta;
    LOG << " sign = " << (int)encoder_write->sign;
    // Return to electrical zero so the rotor is left in a known state.
    voltage_to_timer_pwm(svm.timer, CALIBRATION_VOLTAGE / vbus, 0.0f);
    mcl::sleep_ms(1000);
    // Re-sample zero after returning.
    // This catches cases where the rotor did not settle exactly
    // where it started.
    const uint16_t zero_final = average_encoder_raw(bus);
    const int32_t zero_error = encoder_signed_wrap_delta(encoder_write->zero_raw, zero_final);
    if (zero_error > 20 || zero_error < -20) {
        LOG << "WARNING: encoder zero moved by " << zero_error << " counts";
        return false;
    }
    encoder_write->zero_raw = zero_final;
    LOG << "encoder calibration final zero = "
            << encoder_write->zero_raw << " sign = " << (int)encoder_write->sign;
    *encoder_read = *encoder_write;
    // Discard any velocity-estimator state accumulated during calibration.
    reset_velocity_estimator();
    return true;
}

inline void test_as5600() {
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    while(true) {
        uint16_t raw = as5600_read_raw_angle(bus);
        LOG << " raw: " << raw;
    }
}

#endif