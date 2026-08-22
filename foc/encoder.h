#ifndef ENCODER_H
#define ENCODER_H

// DFRobot 2804, 12N/14P
constexpr uint32_t POLE_PAIRS = 7;
// tune: lower = smoother but more lag
constexpr float VELOCITY_FILTER_ALPHA = 0.2f;

struct encoder_state_t {
    // we want to determine two things
    // Zero — where is the encoder when the motor is at electrical angle 0?
    // Sign — when we command positive electrical rotation, does the encoder count go up or down?
    // +1: encoder increases with positive electrical angle
    // -1: encoder decreases with positive electrical angle
    int8_t sign;
    uint16_t zero_raw;
    uint16_t raw;
    float mechanical_angle = 0.0f;
    float electrical_angle = 0.0f;
    float mechanical_velocity = 0.0f;
    float electrical_velocity = 0.0f;
    uint32_t last_update_cycles = 0;
    bool velocity_valid = false;
};

inline encoder_state_t encoder;

inline uint16_t as5600_read_raw_angle(serial::i2c& bus) {
    //todo:: 4 byte read for now because of 2-byte read bug
    uint8_t buf[4] = {0};
    bus.read(0x36, 0x0C, buf, 4);
    return ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
}

inline uint16_t average_encoder_raw(serial::i2c& bus) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < 100; ++i) {
        sum += as5600_read_raw_angle(bus);
        mcl::sleep_ms(5);
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

inline void calibrate_encoder(serial::i2c& bus, svpwm_t& svm) {
    // Keep the modulation relatively small. It should be large enough to
    // overcome cogging/friction, but not so large that the rotor
    // heats unnecessarily.
    // STEP 1:
    // Force the rotor to electrical angle 0.
    // Valpha = +V
    // Vbeta  =  0
    svpwm_update(svm, 0.15f, 0.0f);
    mcl::sleep_ms(1000);
    encoder.zero_raw = average_encoder_raw(bus);
    LOG << " zero raw = " << encoder.zero_raw;
    // STEP 2:
    // Move the electrical field +90 degrees.
    // Valpha = 0
    // Vbeta  = +V
    // If positive electrical rotation makes the encoder count
    // increase, sign = +1.
    // If it makes the encoder count decrease, sign = -1.
    svpwm_update(svm, 0.0f, 0.15f);
    mcl::sleep_ms(1000);
    const uint16_t plus_90_raw = average_encoder_raw(bus);
    const int32_t delta = encoder_signed_wrap_delta(encoder.zero_raw, plus_90_raw);
    // Determine direction.
    // We expect a meaningful encoder movement. If there is almost
    // no movement, calibration should fail rather than silently
    // choosing a direction.
    constexpr int32_t MIN_DIRECTION_COUNTS = 5;
    if (delta > MIN_DIRECTION_COUNTS) {
        encoder.sign = +1;
    } else if (delta < -MIN_DIRECTION_COUNTS) {
        encoder.sign = -1;
    } else {
        encoder.sign = 0;
    }
    LOG << " +90 raw = " << plus_90_raw;
    LOG << " delta = " << delta;
    LOG << " sign = " << (int)encoder.sign;
    // Return to electrical zero so the rotor is left in a known state.
    svpwm_update(svm, 0.15f, 0.0f);
    mcl::sleep_ms(1000);
    // Re-sample zero after returning.
    // This catches cases where the rotor did not settle exactly
    // where it started.
    const uint16_t zero_final = average_encoder_raw(bus);
    const int32_t zero_error = encoder_signed_wrap_delta(encoder.zero_raw, zero_final);
    if (zero_error > 20 || zero_error < -20) {
        LOG << "WARNING: encoder zero moved by " << zero_error << " counts";
    }
    encoder.zero_raw = zero_final;
    LOG << "encoder calibration final zero = "
            << encoder.zero_raw << " sign = " << (int)encoder.sign;
}

inline float encoder_to_electrical_angle(uint16_t raw,
        float zero_raw, int8_t encoder_sign) {
    constexpr float AS5600_COUNTS = 4096.0f;
    float mechanical =
        ((float)raw - zero_raw) / AS5600_COUNTS;
    // Handle encoder wrap.
    if (mechanical > 0.5f)
        mechanical -= 1.0f;
    else if (mechanical < -0.5f)
        mechanical += 1.0f;
    // Apply measured encoder direction.
    mechanical *= (float)encoder_sign;
    // Convert mechanical revolution to electrical angle.
    float theta = mechanical * TWO_PI * POLE_PAIRS;
    theta = fmodf(theta, TWO_PI);
    if (theta < 0.0f)
        theta += TWO_PI;
    return theta;
}

inline void encoder_read_and_update_angles(serial::i2c& bus) {
    encoder.raw = as5600_read_raw_angle(bus);
    float new_angle = encoder_to_electrical_angle(
            encoder.raw, encoder.zero_raw, encoder.sign);
    uint32_t now_cycles = DWT->CYCCNT;
    if (encoder.velocity_valid) {
        uint32_t delta_cycles = now_cycles - encoder.last_update_cycles;
        float dt = (float)delta_cycles / (float)SystemCoreClock;
        if (dt > 1e-6f) {
            float dtheta = new_angle - encoder.electrical_angle;
            if (dtheta > PI) dtheta -= TWO_PI;
            else if (dtheta < -PI) dtheta += TWO_PI;
            float raw_velocity = dtheta / dt;
            encoder.electrical_velocity +=
                VELOCITY_FILTER_ALPHA * (raw_velocity - encoder.electrical_velocity);
        }
    } else {
        encoder.velocity_valid = true;
    }
    encoder.last_update_cycles = now_cycles;
    encoder.electrical_angle = new_angle;
}

inline void test_as5600() {
    serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
    while(true) {
        uint16_t raw = as5600_read_raw_angle(bus);
        LOG << " raw: " << raw;
        mcl::sleep_ms(1);
    }
}

#endif