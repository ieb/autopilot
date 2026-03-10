// Mock Preferences.h — in-memory NVS replacement for native simulation.

#ifndef HAL_PREFERENCES_H
#define HAL_PREFERENCES_H

#include <stdint.h>
#include <string>
#include <map>

class Preferences {
public:
    void begin(const char* ns, bool readOnly = false) {
        (void)ns; (void)readOnly;
    }
    void end() {}

    uint32_t getULong(const char* key, uint32_t def = 0) {
        auto it = ulong_store.find(key);
        return (it != ulong_store.end()) ? it->second : def;
    }
    void putULong(const char* key, uint32_t val) {
        ulong_store[key] = val;
    }

    float getFloat(const char* key, float def = 0.0f) {
        auto it = float_store.find(key);
        return (it != float_store.end()) ? it->second : def;
    }
    void putFloat(const char* key, float val) {
        float_store[key] = val;
    }

    uint8_t getUChar(const char* key, uint8_t def = 0) {
        auto it = uchar_store.find(key);
        return (it != uchar_store.end()) ? it->second : def;
    }
    void putUChar(const char* key, uint8_t val) {
        uchar_store[key] = val;
    }

private:
    std::map<std::string, uint32_t> ulong_store;
    std::map<std::string, float> float_store;
    std::map<std::string, uint8_t> uchar_store;
};

#endif // HAL_PREFERENCES_H
