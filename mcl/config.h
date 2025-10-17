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
        return getKeyValue(key::action) == 1;
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
};

#endif