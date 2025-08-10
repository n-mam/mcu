#ifndef LOG_H
#define LOG_H

#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <functional>
#include <type_traits>
#include <unordered_map>

// #define _AMD64_
// #include <debugapi.h>

template <typename T, typename... Args>
auto getInstance(Args&&... args) {
    static T s_instance(std::forward<Args>(args)...);
    return &s_instance;
}

namespace mcl {

template<typename T>
using TLogCallback = std::function<void (int, int, const T&)>;

struct log {

    log(){}

    log(int level, int key = log::sink::con, void *target = nullptr) {
        m_key = key;
        m_level = level;
        m_target = target;
    }

    ~log() {
        if (m_target) {
            m_map[m_target](m_level, m_key, a_ss.str());
        } else if (a_sink && a_ss.str().size()) {
            a_sink(m_level, m_key, a_ss.str());
            //OutputDebugStringA(a_ss.str().c_str());
        }
        if (w_sink && w_ss.str().size()) {
            w_sink(m_level, m_key, w_ss.str());
            //OutputDebugStringW(w_ss.str().c_str());
        }
    }
    enum sink {
        con,  // console
        c2c,  // core-core
        cdc,  // usb cdc
        net,  // Network
        spi,  // SPI
        uart  // USART
    };
    enum level {
        debug,
        info,
        warn,
        error,
        status,
        critical
    };

    static void setLogLevel(log::level level) {
        s_app_log_level = level;
    }

    template<typename T>
    static void setLogSink(TLogCallback<T> cbk, void *object = nullptr) {
        if constexpr(std::is_same_v<T, std::string>) {
            if (!object) {
                a_sink = cbk;
            } else {
                m_map[object] = cbk;
            }
        }
        if constexpr(std::is_same_v<T, std::wstring>) {
            w_sink = cbk;
        }
    }

    private:

    template<typename T>
    friend log&& operator <<(log&& lhs, const T& rhs);

    inline static TLogCallback<std::string> a_sink = nullptr;
    inline static TLogCallback<std::wstring> w_sink = nullptr;

    int m_key = 0;
    std::stringstream a_ss;
    std::wstringstream w_ss;
    void *m_target = nullptr;
    int m_level = log::level::info;
    inline static int s_app_log_level = log::level::info;
    inline static std::unordered_map<void *, TLogCallback<std::string>> m_map = {};
};

template<typename T>
log&& operator <<(log&& lhs, const T& rhs) {
    if (lhs.m_level >= log::s_app_log_level) {
        if constexpr(std::is_convertible_v<T, const std::string&>) {
            lhs.a_ss << rhs;
        } else if constexpr(std::is_convertible_v<T, const std::wstring&>) {
            lhs.w_ss << rhs;
        } else if (lhs.a_ss.str().size()) {
            lhs.a_ss << rhs;
        } else {
            lhs.w_ss << rhs;
        }
    }
    return std::move(lhs);
}

inline auto dumpBuffer(const uint8_t *buffer, size_t length) {
    std::ostringstream oss;
    for (size_t i = 0; i < length; i += 16) {
        oss << std::setw(4) << std::setfill('0') << std::hex << i << ": ";
        for (size_t j = 0; j < 16; ++j) {
            if (i + j < length) {
                oss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(buffer[i + j]) << " ";
            } else {
                oss << "   ";
            }
        }
        oss << " | ";
        for (size_t j = 0; j < 16; ++j) {
            if (i + j < length) {
                char ch = static_cast<char>(buffer[i + j]);
                oss << (std::isprint(ch) ? ch : '.');
            } else {
                oss << ' ';
            }
        }
        oss << std::endl;
    }
    return oss.str();
}

}

#define LOG mcl::log(mcl::log::info)
#define DBG mcl::log(mcl::log::debug)
#define WARN mcl::log(mcl::log::warn)
#define ERR mcl::log(mcl::log::error)
#define UART mcl::log(mcl::log::uart)
#define CRITICAL mcl::log(mcl::log::critical)
#define STATUS(s) mcl::log(mcl::log::status, s)
#define PBMSG(s, t) mcl::log(mcl::log::level::info, s, t)

#endif