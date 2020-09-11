#include "ControllerMap.h"


uint8_t ControllerMap::get_i2c_address(uint8_t nucleo, uint8_t channel) {
    return ((nucleo + 1) << 4) | channel;
}

void ControllerMap::init() {
    name_map["HAND_FINGER_POS"] = get_i2c_address(0, 2);
    name_map["HAND_FINGER_NEG"] = get_i2c_address(0, 3);
    name_map["HAND_GRIP_POS"] = get_i2c_address(0, 4);
    name_map["HAND_GRIP_NEG"] = get_i2c_address(0, 5);
    name_map["FOOT_CLAW"] = get_i2c_address(2, 0);
    name_map["FOOT_SENSOR"] = get_i2c_address(1, 1);
    name_map["GIMBAL_PITCH_0_POS"] = get_i2c_address(1, 2);
    name_map["GIMBAL_PITCH_0_NEG"] = get_i2c_address(1, 3);
    name_map["GIMBAL_PITCH_1_POS"] = get_i2c_address(2, 2);
    name_map["GIMBAL_PITCH_1_NEG"] = get_i2c_address(2, 3);
    name_map["GIMBAL_YAW_0_POS"] = get_i2c_address(1, 4);
    name_map["GIMBAL_YAW_0_NEG"] = get_i2c_address(1, 5);
    name_map["GIMBAL_YAW_1_POS"] = get_i2c_address(2, 4);
    name_map["GIMBAL_YAW_1_NEG"] = get_i2c_address(2, 5);
    name_map["SA_0"] = get_i2c_address(0, 0);
    name_map["SA_1"] = get_i2c_address(0, 1);
    name_map["SA_2"] = get_i2c_address(1, 0);
    name_map["RA_0"] = get_i2c_address(0, 0);
    name_map["RA_1"] = get_i2c_address(0, 1);
    name_map["RA_2"] = get_i2c_address(1, 0);
    name_map["RA_3"] = get_i2c_address(1, 1);
    name_map["RA_4"] = get_i2c_address(2, 0);
    name_map["RA_5"] = get_i2c_address(2, 1);
}


uint8_t ControllerMap::get_i2c_address(std::string name) {
    return name_map[name];
}

bool ControllerMap::check_if_live(std::string name) {
    return (name == live_map[get_i2c_address(name)]);
}

void ControllerMap::make_live(std::string name)
{
    live_map[get_i2c_address(name)] = name;
}