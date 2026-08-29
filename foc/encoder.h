#ifndef ENCODER_H
#define ENCODER_H

// DFRobot 2804, 12N/14P
constexpr uint32_t POLE_PAIRS = 7;
constexpr float AS5600_COUNTS = 4096.0f;
inline constexpr uint32_t ENCODER_READ_PERIOD_US = 1'000; // 1 kHz
// tune: lower = smoother but more lag
constexpr float AS5600_COUNT_TO_RAD = TWO_PI / AS5600_COUNTS;

struct encoder_state_t {
    // Calibration
    // we want to determine two things
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

// M/T-style velocity estimator for the AS5600 (absolute, I2C, 12-bit).
// WHY: computing delta_angle / delta_time on every single encoder read
// is dominated by +/-1 LSB quantization noise whenever dt happens to be
// short (which it often is in a busy foreground loop with no fixed
// sample period). One AS5600 count is ~1.53 mrad mechanical; dividing
// that by a sub-millisecond dt gives velocity spikes of many rad/s, and
// zero-count reads give exact-zero readings. That's the noisy,
// sign-flipping "we"/"wm" you're seeing in the log.
//
// FIX: instead of computing a new velocity sample on every read,
// accumulate the (unwrapped) angle delta across reads and only emit a
// new velocity estimate once the accumulated rotation exceeds a
// meaningful threshold (a handful of counts) -- or a max-wait timeout
// expires, so the estimate still updates at near-zero speed instead of
// freezing. The elapsed time is still measured precisely via
// DWT->CYCCNT, so there's no timing quantization -- only the position
// quantization remains, and it's now a small fraction of a much larger
// numerator, so the relative noise drops a lot.

struct mt_velocity_estimator_t {
    // Accumulated signed AS5600 counts since the last
    // emitted velocity estimate.
    int32_t accumulated_counts = 0;
    // DWT cycle count at the beginning of the current
    // measurement window.
    uint32_t window_start_cycles = 0;
    // Emit a new velocity estimate after this many counts.
    int32_t min_count_threshold = 3;
    // Force an update at very low speed so velocity does
    // not remain stale forever.
    float max_wait_s = 0.05f;
    bool initialized = false;
};

inline encoder_state_t encoder_a;
inline encoder_state_t encoder_b;
inline encoder_state_t* encoder_write = &encoder_a;
inline encoder_state_t* encoder_read  = &encoder_b;
inline mt_velocity_estimator_t g_mt_vel;

inline void reset_velocity_estimator() {
    g_mt_vel = {};
    encoder_a.mechanical_velocity = 0.0f;
    encoder_a.electrical_velocity = 0.0f;
    encoder_a.previous_raw = encoder_a.raw;
    encoder_b.mechanical_velocity = 0.0f;
    encoder_b.electrical_velocity = 0.0f;
    encoder_b.previous_raw = encoder_b.raw;
    encoder_a.last_update_cycles = DWT->CYCCNT;
    encoder_b.last_update_cycles = encoder_a.last_update_cycles;
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

// Call this once per encoder read with the *unwrapped* mechanical angle
// delta since the previous call (same +/-PI unwrap logic you already
// have for delta_angle). It internally decides whether enough new
// information has accumulated to actually update
// encoder.mechanical_velocity / encoder.electrical_velocity.
inline void mt_velocity_update(int32_t delta_counts) {

    const uint32_t now_cycles = DWT->CYCCNT;

    if (!g_mt_vel.initialized) {
        g_mt_vel.initialized = true;
        g_mt_vel.window_start_cycles = now_cycles;
        g_mt_vel.accumulated_counts = 0;
        return;
    }

    g_mt_vel.accumulated_counts += delta_counts;

    const uint32_t elapsed_cycles =
        now_cycles - g_mt_vel.window_start_cycles;

    const float elapsed_s =
        (float)elapsed_cycles / (float)SystemCoreClock;

    const bool count_threshold_met =
        abs(g_mt_vel.accumulated_counts) >= g_mt_vel.min_count_threshold;

    const bool timed_out = elapsed_s >= g_mt_vel.max_wait_s;

    if (!count_threshold_met && !timed_out) return;

    if (elapsed_s > 1e-5f) {
        float velocity_raw = 0.0f;
        // At timeout, if there has been no encoder movement,
        // explicitly drive the estimate toward zero.
        if (timed_out && g_mt_vel.accumulated_counts == 0) {
            velocity_raw = 0.0f;
        } else {
            const float accumulated_angle =
                (float)g_mt_vel.accumulated_counts * AS5600_COUNT_TO_RAD;
            velocity_raw = accumulated_angle / elapsed_s;
        }

        constexpr float VELOCITY_ALPHA = 0.3f;

        encoder_write->mechanical_velocity =
            encoder_write->mechanical_velocity +
            VELOCITY_ALPHA *
                (velocity_raw - encoder_write->mechanical_velocity);

        encoder_write->electrical_velocity =
            encoder_write->mechanical_velocity * (float)POLE_PAIRS;
        // LOG << "VEL" << " dt_us=" << (elapsed_s * 1e6f) << " counts=" << g_mt_vel.accumulated_counts
        //         << " vraw=" << velocity_raw << " wm=" << encoder.mechanical_velocity
        //             << " we=" << encoder.electrical_velocity;
    }
    // Start a new measurement window.
    g_mt_vel.accumulated_counts = 0;
    g_mt_vel.window_start_cycles = now_cycles;
}

// Extrapolate encoder state forward from the last I2C sample using
// the last measured velocity/acceleration. Called from the ADC ISR,
// which runs far more often than the encoder is actually read.
inline float encoder_predict_electrical_angle(
    const encoder_state_t& encoder_state) {

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

    // Velocity estimation.
    // Use raw AS5600 counts directly.
    const int32_t delta_counts =
        encoder_signed_wrap_delta(
            encoder_write->previous_raw,
            encoder_write->raw) *
        (int32_t)encoder_write->sign;

    mt_velocity_update(delta_counts);

    encoder_write->previous_raw = encoder_write->raw;
    encoder_write->last_update_cycles = DWT->CYCCNT;

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
    svpwm_update(svm, CALIBRATION_VOLTAGE / vbus, 0.0f);
    mcl::sleep_ms(1000);
    encoder_write->zero_raw = average_encoder_raw(bus);
    LOG << " zero raw = " << encoder_write->zero_raw;
    // Move the electrical field +90 degrees.
    // Valpha = 0
    // Vbeta  = +V
    // If positive electrical rotation makes the encoder count
    // increase, sign = +1.
    // If it makes the encoder count decrease, sign = -1.
    svpwm_update(svm, 0.0f, CALIBRATION_VOLTAGE / vbus);
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
    svpwm_update(svm, 0.15f, 0.0f);
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