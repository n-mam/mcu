#ifndef TRACE_H
#define TRACE_H

struct velocity_trace_sample_t {
    uint16_t raw;
    int16_t  delta_counts;
    float dt_s;
    float raw_velocity;
    float filtered_velocity;
    float electrical_angle;
};

constexpr uint32_t TRACE_LEN = 2048; // ~2s of samples at 1kHz
inline velocity_trace_sample_t g_trace[TRACE_LEN];
inline uint32_t g_trace_index = 0;
inline bool g_trace_capturing = false;
inline bool g_trace_full = false;

inline void trace_start() {
    g_trace_index = 0;
    g_trace_full = false;
    g_trace_capturing = true;
}

inline void trace_push(uint16_t raw, int32_t delta_counts, float dt_s,
                        float raw_velocity, float filtered_velocity,
                        float electrical_angle) {
    if (!g_trace_capturing) return;
    auto& s = g_trace[g_trace_index];
    s = { raw, (int16_t)delta_counts, dt_s, raw_velocity,
          filtered_velocity, electrical_angle };
    if (++g_trace_index >= TRACE_LEN) {
        g_trace_index = 0;
        g_trace_full = true;
        g_trace_capturing = false; // one-shot; call trace_start() to rearm
    }
}

inline void trace_dump() {
    for (uint32_t i = 0; i < TRACE_LEN; ++i) {
        const auto& s = g_trace[i];
        LOG << "TR " << i << " raw:" << s.raw << " dcnt:" << s.delta_counts
            << " dt_us:" << (s.dt_s * 1e6f) << " vraw:" << s.raw_velocity
            << " vfilt:" << s.filtered_velocity << " theta:" << s.electrical_angle;
    }
}

#endif
