#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <vector>
#include <string>
#include <functional>

#include <mcl/log.h>

struct command {
    enum class command_type {
        kv, action, invalid,
    };
    KeyValue kv{};
    std::string action;
    command_type type =
        command_type::invalid;
};


struct config {

    enum key : uint8_t {
        vf_mod,
        vf_ef,
        s_ref,
        s_kp,
        s_ki,
        iq_ref,
        id_ref,
        c_kp,
        c_ki,
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

    inline bool parseCommand(const std::string& raw, command& cmd) {
        const auto separator = raw.find(':');
        if (separator == std::string::npos) {
            cmd.type = command::command_type::action;
            cmd.action = raw;
            return true;
        }
        const std::string name = raw.substr(0, separator);
        const std::string value = raw.substr(separator + 1);
        LOG << "name: " << name << " value: " << value;
        const auto it = names_key.find(name);
        if (it == names_key.end()) return false;
        char* end = nullptr;
        const float parsedValue = std::strtof(value.c_str(), &end);
        if (end == value.c_str() || *end != '\0') return false;
        cmd.kv.key = static_cast<uint32_t>(it->second);
        cmd.kv.value = parsedValue;
        cmd.type = command::command_type::kv;
        getInstance<config>()->setKeyValue(cmd.kv);
        return true;
    }

    private:

    std::map<config::key, float> key_values;

    std::map<config::key, std::string> key_names = {
        {key::vf_mod, "vf_mod"},
        {key::vf_ef, "vf_ef"},
        {key::s_ki, "s_ki"},
        {key::s_kp, "s_kp"},
        {key::s_ref, "s_ref"},
        {key::id_ref, "id_ref"},
        {key::iq_ref, "iq_ref"},
        {key::c_ki, "c_ki"},
        {key::c_kp, "c_kp"},
        {key::action, "action"}
    };

    std::map<std::string, key> names_key = {
        {"vf_mod", key::vf_mod},
        {"vf_ef", key::vf_ef},
        {"s_ki", key::s_ki},
        {"s_kp", key::s_kp},
        {"s_ref", key::s_ref},
        {"id_ref", key::id_ref},
        {"iq_ref", key::iq_ref},
        {"c_ki", key::c_ki},
        {"c_kp", key::c_kp},
        {"action", key::action}
    };
};

#endif