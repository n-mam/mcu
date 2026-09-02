#ifndef FOC_H
#define FOC_H

#include <foc/hardware.h>
#include <foc/svpwm.h>
#include <foc/encoder.h>
#include <foc/current.h>
#include <foc/speed.h>

struct foc_controller_t {

    hardware_t hw{};
    encoder_t encoder;
    speed_control_t sc;
    current_control_t cc;
    static foc_controller_t *active;

    enum foc_state { stopped, running, fault };
    foc_state state = foc_state::stopped;

    void start() {
        active = this;
        // TIM1 CH1/2/3 SVPWM setup
        hw.initialize_phase_pwm_timer(nullptr);
        // CSA ADC setup
        hw.initialize_adc_hardware(&adc_trampoline);
        // Disbale till we calibrate CSA and the encoder
        state = foc_state::stopped;
        // Zero current ADC bias calibration
        hw.calibrate_current_sensor();
        // Calibrate encoder's sign and zero offset
        encoder.calibrate_sign_and_offset(&hw.pwm_timer);
        // Encoder TIM2 read setup
        hw.initialize_encoder_timer(&encoder_trampoline);
        // With vbus = 9.49 V
        // v_limit = 9.49 / sqrt(3) ≈ 5.48 V
        // so each PI can request up to ±5.48 V.
        const float v_limit = SVPWM_MAX_MODULATION * VBUS;
        cc.d_pi = { .kp = Kp, .ki = Ki, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
        cc.q_pi = { .kp = Kp, .ki = Ki, .integrator = 0.0f, .out_min = -v_limit, .out_max = v_limit };
        sc.pi   = { .kp = 0.03f, .ki = 1.4f, .integrator = 0.0f, .out_min = -0.30f, .out_max = 0.30f };
        // manual hold delay
        mcl::delay_ms(2000);
        cc.reset();
        sc.reset();
        state = foc_state::running;

        uint64_t total_cycles = 0;
        uint32_t last_cycles = DWT->CYCCNT;
        uint64_t last_log_cycles = 0;
        uint64_t log_period_cycles = (uint64_t)SystemCoreClock / 2U;

        // cc.d_ref = 0.0f;
        // cc.q_ref = 0.3f;
        constexpr float SPEED_START = 15.0f;
        float SPEED_END = 1.0f; //0.5f; // 0.25f; OK
        constexpr float SPEED_RAMP_DURATION_S = 30.0f;
        sc.speed_ref = SPEED_START;
        float inv_SystemCoreClock = 1 / (float)SystemCoreClock;
        uint32_t last_ramp_ms = mcl::time_ms();

        while (state == foc_state::running) {
            uint32_t now_cycles = DWT->CYCCNT;
            total_cycles += (uint32_t)(now_cycles - last_cycles);
            last_cycles = now_cycles;
            float elapsed_s = (float)total_cycles * inv_SystemCoreClock;
            uint32_t now_ms = mcl::time_ms();

            if ((uint32_t)(now_ms - last_ramp_ms) >= 1U) {
                sc.speed_ref = ramp_linear(
                    SPEED_START,
                    SPEED_END,
                    SPEED_RAMP_DURATION_S,
                    elapsed_s);
                last_ramp_ms = now_ms;
            }

            if (total_cycles - last_log_cycles >= log_period_cycles) {
                last_log_cycles = total_cycles;
                log_foc_state(elapsed_s);
            }
        }
        timer_stop(&hw.pwm_timer);
    }

    void stop() {
        state = foc_state::stopped;
    }

    static void adc_trampoline(uint16_t a, uint16_t b) {
        if (active) active->adc_callback(a, b);
    }

    static void encoder_trampoline(timer_event_t ev) {
        if (active) active->encoder_callback(ev);
    }

    void adc_callback(uint16_t raw_a, uint16_t raw_b) {
        // Disable FOC during encoder/zero current calibration
        if (state != foc_state::running) {
            hw.update_current_calibration_sample(raw_a, raw_b);
            return;
        }
        // ADC counts to phase currents
        cc.adc_to_phase_currents(raw_a, raw_b, hw.bias_a, hw.bias_b);
        // Snapshot read encoder
        const auto& enc = encoder.read();
        // Predicted angle
        const auto electrical_angle = encoder.predict_electrical_angle();
        // Speed control
        static uint16_t speed_loop_divider = 0;
        if (++speed_loop_divider >= 10) {
            speed_loop_divider = 0;
            constexpr float SPEED_LOOP_DT = 0.0005f;
            cc.q_ref = sc.speed_control(enc.mechanical_velocity, SPEED_LOOP_DT);
        }
        // Current control
        constexpr float CURRENT_LOOP_DT = 1.0f / 20'000.0f;
        auto dq = cc.current_control(electrical_angle,
                enc.electrical_velocity, CURRENT_LOOP_DT);

        auto ab = inverse_park_transform(dq, electrical_angle);
        // PI outputs are in volts; svpwm
        // expects Vref normalized to Vbus.
        auto out = voltage_to_timer_pwm(&(hw.pwm_timer),
            ab.alpha * INV_VBUS, ab.beta * INV_VBUS);
        cc.modulation = out.modulation;
    }

    void encoder_callback(timer_event_t event) {
        if (event == TIMER_EVENT_UPDATE) {
            // executes at 1kHz
            encoder.update();
        }
    }

    inline void log_foc_state(float elapsed_s) {
        // Snapshot read encoder
        const auto& enc = encoder.read();
        char log_buffer[512];
        int len = snprintf(log_buffer, sizeof(log_buffer),
            "theta:%f elapsed:%f s_ref:%f s_mes:%f "
            "[ia:%f ib:%f ic:%f] q_ref:%fA [id:%f iq:%f] [vd:%f vq:%f] mod:%f vd_i:%f vq_i:%f s_i:%f",
                enc.electrical_angle, elapsed_s, sc.speed_ref, sc.speed_measured,
                    cc.ia, cc.ib, cc.ic, cc.q_ref, cc.id, cc.iq, cc.vd, cc.vq,
                        cc.modulation, cc.d_pi.integrator, cc.q_pi.integrator, sc.pi.integrator);
        if (len > 0) {
            if (len >= sizeof(log_buffer)) len = sizeof(log_buffer) - 1;
            LOG << std::string(log_buffer, len);
        }
    }
};

foc_controller_t *foc_controller_t::active = nullptr;

inline void test_foc() {
    static foc_controller_t foc{};
    foc.start();
}

#endif