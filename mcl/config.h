#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <vector>
#include <string>
#include <functional>

#include <mcl/log.h>

struct config {

    enum key : uint8_t {
        ki,
        kp,
        kd,
        delay,
        motor,
        servo,
        action,
    };

    std::string peer_ip;

    inline auto getKeyName(key k) {
        return key_names[k];
    }

    inline auto getKeyValue(key k) {
        return key_values[k];
    }

    inline auto setKeyValue(const KeyValue& kv) {
        LOG << "k: " << getKeyName(static_cast<key>(kv.key)) << ", v: " << kv.value;
        key_values[static_cast<config::key>(kv.key)] = static_cast<float>(kv.value);
    }

    inline auto shouldExit() {
        return getKeyValue(key::action) == -1;
    }

    inline bool parseCommand(const std::string& command) {
        const auto separator = command.find(':');
        if (separator == std::string::npos) return false;
        const std::string name = command.substr(0, separator);
        const std::string value = command.substr(separator + 1);
        const auto it = key_lookup.find(name);
        if (it == key_lookup.end()) return false;
        char* end = nullptr;
        const float parsedValue = std::strtof(value.c_str(), &end);
        if (end == value.c_str() || *end != '\0') return false;
        KeyValue kv;
        kv.key = static_cast<uint32_t>(it->second);
        kv.value = parsedValue;
        LOG << "k:" << kv.key << " v:" << kv.value;
        getInstance<config>()->setKeyValue(kv);
        return true;
    }

    private:

    std::map<config::key, float> key_values;

    std::map<config::key, std::string> key_names = {
        {key::kp, "kp"},
        {key::ki, "ki"},
        {key::kd, "kd"},
        {key::motor, "motor"},
        {key::servo, "servo"},
        {key::delay, "delay"},
        {key::action, "action"}
    };

    std::map<std::string, key> key_lookup = {
        {"ki",     key::ki},
        {"kp",     key::kp},
        {"kd",     key::kd},
        {"delay",  key::delay},
        {"motor",  key::motor},
        {"servo",  key::servo},
        {"action", key::action}
    };
};

#endif