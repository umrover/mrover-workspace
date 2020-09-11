#ifndef CONTROLLER_MAP_H
#define CONTROLLER_MAP_H
#include <stdint.h>
#include <string>
#include <unordered_map>

class ControllerMap
{
private:
    inline static std::unordered_map<uint8_t, std::string> live_map = std::unordered_map<uint8_t, std::string>();
    inline static std::unordered_map<std::string, uint8_t> name_map = std::unordered_map<std::string, uint8_t>();

    static uint8_t get_i2c_address(uint8_t nucleo, uint8_t channel);

public:
    static void init();

    static uint8_t get_i2c_address(std::string name);

    static bool check_if_live(std::string name);

    static void make_live(std::string name);
};

#endif