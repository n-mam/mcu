#ifndef ENCODER_H
#define ENCODER_H

// DFRobot 2804, 12N/14P
constexpr uint32_t POLE_PAIRS = 7;
constexpr float AS5600_COUNTS = 4096.0f;
constexpr float AS5600_COUNT_TO_RAD = TWO_PI / AS5600_COUNTS;

struct encoder_state_t {
    // Latest raw AS5600 reading
    uint16_t raw = 0;
    // Position
    // Mechanical rotor angle in radians
    // Range: approximately [-PI, PI]
    float mechanical_angle = 0.0f;
    // Electrical angle used by the Park transform
    // Range: [0, TWO_PI)
    float electrical_angle = 0.0f;
    // Velocity
    // Mechanical angular velocity in rad/s
    float mechanical_velocity = 0.0f;
    // Electrical angular velocity in rad/s
    // = mechanical_velocity * POLE_PAIRS
    float electrical_velocity = 0.0f;
    // Timestamp of this published encoder sample
    uint32_t timestamp_cycles = 0;
};

struct encoder_t {
    // Calibration: we want to determine two things
    // Zero — where is the encoder when the motor is at electrical angle 0 ? and
    // Sign — when we command positive electrical rotation, does the encoder count go up or down?
    // +1: encoder count increases with positive electrical rotation
    // -1: encoder count decreases with positive electrical rotation
    int8_t sign = 1;
    // AS5600 raw count corresponding to electrical zero
    uint16_t zero_raw = 0;
    // Double-buffered encoder state
    encoder_state_t state_a{};
    encoder_state_t state_b{};
    encoder_state_t* read_state = &state_a;
    encoder_state_t* write_state = &state_b;
    // Velocity estimator state
    uint16_t previous_raw = 0;
    uint32_t last_update_cycles = 0;
    float filtered_velocity = 0.0f;
    float velocity_time_constant_s = 0.005f;
    float raw_velocity = 0.0f;
    bool velocity_initialized = false;
    serial::i2c *encoder_i2c = nullptr;

    // State access
    inline const encoder_state_t& read() const {
        return *read_state;
    }

    inline uint16_t read_raw(serial::i2c& bus) {
        // 2-byte reads on F446
        // should not be interruptible
        uint8_t buf[3] = {0};
        //__disable_irq();
        bus.read(0x36, 0x0C, buf, 3);
        //__enable_irq();
        return ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    }

    inline uint16_t average_raw(serial::i2c& bus) {
        uint64_t sum = 0;
        for (uint32_t i = 0; i < 100; ++i) {
            sum += read_raw(bus);
            mcl::sleep_ms(1);
        }
        return (uint16_t)(sum / 100);
    }

    // calculate the shortest signed difference between two
    // AS5600 raw angles, accounting for the 0/4095 wraparound.
    inline int32_t signed_wrap_delta(uint16_t from, uint16_t to) {
        // AS5600 is 12-bit: 0..4095.
        // Return shortest signed difference in [-2048, +2047].
        int32_t delta = (int32_t)to - (int32_t)from;
        if (delta > 2048) {
            delta -= 4096;
        } else if (delta < -2048) {
            delta += 4096;
        }
        return delta;
    }

    inline float raw_to_mechanical_angle(uint16_t raw) {
        float angle = ((float)raw -
            (float)zero_raw) / AS5600_COUNTS;
        // wrap to [-0.5, +0.5] mechanical turns.
        if (angle > 0.5f) {
            angle -= 1.0f;
        } else if (angle < -0.5f) {
            angle += 1.0f;
        }
        angle *= (float)sign;
        return angle * TWO_PI;
    }

    // Velocity estimator
    inline void reset_velocity() {
        filtered_velocity = 0.0f;
        raw_velocity = 0.0f;
        velocity_initialized = false;
        previous_raw = read_state->raw;
        const uint32_t now = DWT->CYCCNT;
        last_update_cycles = now;
        state_a.mechanical_velocity = 0.0f;
        state_a.electrical_velocity = 0.0f;
        state_a.timestamp_cycles = now;
        state_b.mechanical_velocity = 0.0f;
        state_b.electrical_velocity = 0.0f;
        state_b.timestamp_cycles = now;
    }

    // Plain per-tick finite difference + first-order low-pass, velocity
    // estimator. the longer filter tau does the noise/lag tradeoff work
    inline void update_velocity(int32_t delta_counts, float dt_s) {

        if (!velocity_initialized || dt_s <= 1e-6f) {
            velocity_initialized = true;
            raw_velocity = 0.0f;
            filtered_velocity = 0.0f;
            write_state->mechanical_velocity = 0.0f;
            write_state->electrical_velocity = 0.0f;
            return;
        }

        const float angle_delta = (float)delta_counts * AS5600_COUNT_TO_RAD;
        raw_velocity = angle_delta / dt_s;
        // First-order low-pass filter.
        const float alpha = dt_s / (velocity_time_constant_s + dt_s);
        filtered_velocity +=
            alpha * (raw_velocity - filtered_velocity);

        write_state->mechanical_velocity = filtered_velocity;
        write_state->electrical_velocity =
            filtered_velocity * (float)POLE_PAIRS;
    }

    // Extrapolate encoder state forward from the published
    // encoder sample using the latest measured velocity..
    // Called from the ADC ISR, which runs far more often
    // than the encoder is actually read.
    inline float predict_electrical_angle() const {
        const encoder_state_t& state = *read_state;
        const float encoder_age =
            (float)(DWT->CYCCNT - state.timestamp_cycles) /
                (float)SystemCoreClock;
        float electrical_angle = state.electrical_angle +
            state.electrical_velocity * encoder_age;
        electrical_angle = fmodf(electrical_angle, TWO_PI);
        if (electrical_angle < 0.0f) {
            electrical_angle += TWO_PI;
        }
        return electrical_angle;
    }

    // Encoder update
    inline void update() {
        // Start from the last published snapshot so the write buffer
        // contains a complete, coherent state before modifying it.
        *write_state = *read_state;
        // Read AS5600
        write_state->raw = read_raw(*encoder_i2c);
        // Mechanical angle
        write_state->mechanical_angle =
            raw_to_mechanical_angle(write_state->raw);
        // Electrical angle
        float electrical_angle =
            write_state->mechanical_angle * (float)POLE_PAIRS;

        electrical_angle = fmodf(electrical_angle, TWO_PI);

        if (electrical_angle < 0.0f) {
            electrical_angle += TWO_PI;
        }

        write_state->electrical_angle = electrical_angle;

        // Velocity
        const int32_t delta_counts = signed_wrap_delta
                (previous_raw, write_state->raw) * (int32_t)sign;

        const uint32_t now_cycles = DWT->CYCCNT;

        const float dt_s =
            (float)(now_cycles - last_update_cycles) /
                (float)SystemCoreClock;

        update_velocity(delta_counts, dt_s);
        previous_raw = write_state->raw;
        last_update_cycles = now_cycles;
        write_state->timestamp_cycles = now_cycles;
        // Publish
        encoder_state_t* tmp = read_state;
        read_state = write_state;
        write_state = tmp;
    }

    inline bool calibrate_sign_and_offset(timer_config_t *timer) {
        // Encoder sign and electrical-zero calibration
        encoder_i2c = new serial::i2c(3, 10, 400'000, I2C2, GPIOB);
        auto& bus = *encoder_i2c;
        constexpr float CALIBRATION_VOLTAGE = 1.5f;
        // Force rotor to electrical angle 0.
        // Valpha = +V
        // Vbeta  =  0
        voltage_to_timer_pwm(timer, CALIBRATION_VOLTAGE / VBUS, 0.0f);
        mcl::sleep_ms(1000);
        const uint16_t initial_zero_raw = average_raw(bus);
        LOG << " zero raw = " << initial_zero_raw;
        // Move the electrical field +90 degrees.
        // Valpha = 0
        // Vbeta  = +V
        // If positive electrical rotation makes the encoder count
        // increase, sign = +1.
        // If it makes the encoder count decrease, sign = -1.
        voltage_to_timer_pwm(timer, 0.0f, CALIBRATION_VOLTAGE / VBUS);
        mcl::sleep_ms(1000);
        const uint16_t plus_90_raw = average_raw(bus);
        const int32_t delta = signed_wrap_delta(initial_zero_raw, plus_90_raw);
        // Determine direction.
        // We expect a meaningful encoder movement. If there is almost
        // no movement, calibration should fail rather than silently
        // choosing a direction.
        constexpr int32_t MIN_DIRECTION_COUNTS = 5;
        if (delta > MIN_DIRECTION_COUNTS) {
            sign = +1;
        } else if (delta < -MIN_DIRECTION_COUNTS) {
            sign = -1;
        } else {
            sign = 0;
            return false;
        }
        LOG << " +90 raw = " << plus_90_raw;
        LOG << " delta = " << delta;
        LOG << " sign = " << (int)sign;
        // Return to electrical zero so the rotor is left in a known state.
        voltage_to_timer_pwm(timer, CALIBRATION_VOLTAGE / VBUS, 0.0f);
        mcl::sleep_ms(1000);
        // Re-sample zero.
        // This catches cases where the rotor did not settle
        // exactly where it started.
        const uint16_t final_zero_raw = average_raw(bus);
        const int32_t zero_error = signed_wrap_delta(initial_zero_raw, final_zero_raw);
        if (zero_error > 20 || zero_error < -20) {
            LOG << "WARNING: encoder zero moved by " << zero_error << " counts";
            return false;
        }
        zero_raw = final_zero_raw;
        LOG << "encoder calibration final zero = "
                << zero_raw << " sign = " << (int)sign;
        // Reset runtime state.
        state_a = {};
        state_b = {};
        state_a.raw = final_zero_raw;
        state_b.raw = final_zero_raw;
        state_a.electrical_angle = 0.0f;
        state_b.electrical_angle = 0.0f;
        const uint32_t now = DWT->CYCCNT;
        state_a.timestamp_cycles = now;
        state_b.timestamp_cycles = now;
        read_state = &state_a;
        write_state = &state_b;
        previous_raw = final_zero_raw;
        last_update_cycles = now;
        reset_velocity();
        return true;
    }

    inline void test_as5600() {
        serial::i2c bus(3, 10, 400'000, I2C2, GPIOB);
        while(true) {
            const uint16_t raw = read_raw(bus);
            LOG << " raw: " << raw;
        }
    }
};

#endif