#pragma once

#include "mbed.h"
#include "math_3d.hpp"

class Imu {
    public:
        Imu(PinName sda, PinName scl);

        void init();

        bool valid() const;

        void read();

        Math::Vector3f accelerometer() const;
        Math::Vector3f magnetometer() const;
        Math::Vector3f gyroscope() const;

    private:
        // Address and self-reported ID of accelerometer/magnetometer
        static const uint8_t AM_ADDRESS = 0x3E;
        static const uint8_t AM_ID = 0xC7;

        // Address and self-reported ID of gyroscope
        static const uint8_t GY_ADDRESS = 0x42;
        static const uint8_t GY_ID = 0xD7;

        // Registers for accelerometer/magnetometer
        static const uint8_t AM_REGISTER_STATUS = 0x00;
        static const uint8_t AM_REGISTER_OUT_X_MSB = 0x01;
        static const uint8_t AM_REGISTER_OUT_X_LSB = 0x02;
        static const uint8_t AM_REGISTER_OUT_Y_MSB = 0x03;
        static const uint8_t AM_REGISTER_OUT_Y_LSB = 0x04;
        static const uint8_t AM_REGISTER_OUT_Z_MSB = 0x05;
        static const uint8_t AM_REGISTER_OUT_Z_LSB = 0x06;
        static const uint8_t AM_REGISTER_WHO_AM_I = 0x0D;
        static const uint8_t AM_REGISTER_XYZ_DATA_CFG = 0x0E;
        static const uint8_t AM_REGISTER_CTRL_REG1 = 0x2A;
        static const uint8_t AM_REGISTER_CTRL_REG2 = 0x2B;
        static const uint8_t AM_REGISTER_CTRL_REG3 = 0x2C;
        static const uint8_t AM_REGISTER_CTRL_REG4 = 0x2D;
        static const uint8_t AM_REGISTER_CTRL_REG5 = 0x2E;
        static const uint8_t AM_REGISTER_MSTATUS = 0x32;
        static const uint8_t AM_REGISTER_MOUT_X_MSB = 0x33;
        static const uint8_t AM_REGISTER_MOUT_X_LSB = 0x34;
        static const uint8_t AM_REGISTER_MOUT_Y_MSB = 0x35;
        static const uint8_t AM_REGISTER_MOUT_Y_LSB = 0x36;
        static const uint8_t AM_REGISTER_MOUT_Z_MSB = 0x37;
        static const uint8_t AM_REGISTER_MOUT_Z_LSB = 0x38;
        static const uint8_t AM_REGISTER_MCTRL_REG1 = 0x5B;
        static const uint8_t AM_REGISTER_MCTRL_REG2 = 0x5C;
        static const uint8_t AM_REGISTER_MCTRL_REG3 = 0x5D;

        // Registers for gyroscope
        static const uint8_t GY_REGISTER_STATUS = 0x00;
        static const uint8_t GY_REGISTER_OUT_X_MSB = 0x01;
        static const uint8_t GY_REGISTER_OUT_X_LSB = 0x02;
        static const uint8_t GY_REGISTER_OUT_Y_MSB = 0x03;
        static const uint8_t GY_REGISTER_OUT_Y_LSB = 0x04;
        static const uint8_t GY_REGISTER_OUT_Z_MSB = 0x05;
        static const uint8_t GY_REGISTER_OUT_Z_LSB = 0x06;
        static const uint8_t GY_REGISTER_WHO_AM_I = 0x0C;
        static const uint8_t GY_REGISTER_CTRL_REG0 = 0x0D;
        static const uint8_t GY_REGISTER_CTRL_REG1 = 0x13;
        static const uint8_t GY_REGISTER_CTRL_REG2 = 0x14;
        
        // Unit conversions
        static const float ACCEL_MG_LSB_4G = 0.000488f;
        static const float MAG_UT_LSB = 0.1f;
        static const float GYRO_SENSITIVITY_1000DPS = 0.03125f;
        static const float GRAVITY_STANDARD = 9.80665f;
        static const float DPS_TO_RADS = 0.017453293f;

        void write_am_reg(uint8_t reg, uint8_t value);
        uint8_t read_am_reg(uint8_t reg);

        void write_gy_reg(uint8_t reg, uint8_t value);
        uint8_t read_gy_reg(uint8_t reg);

        void write_reg(uint8_t addr, uint8_t reg, uint8_t value);
        uint8_t read_reg(uint8_t addr, uint8_t reg);

        I2C i2c_;

        Math::Vector3f accel_;
        Math::Vector3f mag_;
        Math::Vector3f gyro_;

        bool valid_;
        bool check_valid_;
};
