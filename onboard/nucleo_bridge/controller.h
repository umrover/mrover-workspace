#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstring>
#include <vector>
#include <cmath>
#include <mutex>
#include <limits>
#include "hardware.h"
#include "backend.h"

#define OFF 0x00, 0, 0
#define ON 0x0F, 0, 0
#define OPEN 0x10, 2, 0
#define OPEN_PLUS 0x1F, 2, 4
#define CLOSED 0x20, 8, 0
#define CLOSED_PLUS 0x2F, 8, 4
#define CONFIG_PWM 0x30, 6, 0
#define CONFIG_K 0x3F, 12, 0
#define QUAD 0x40, 0, 4
#define ADJUST 0x4F, 4, 0
#define SPI 0x50, 0, 2
#define LIMIT 0x60 0, 1

#define POINTER reinterpret_cast<uint8_t *>

class Controller {
public:
    float startAngle = 0.0;
    float torqueScale = 1.0;
    float quadCPR = std::numeric_limits<float>::infinity();
    float spiCPR = std::numeric_limits<float>::infinity();
    float currentAngle = 0.0;
    float kP, kI, kD = 0.0;

    BackEnd *backEnd;
    std::string name;

private:    
    uint8_t address;
    Hardware hardware;
    std::mutex controllerMutex;

    bool active() {
        return name == backEnd->getHardwares(address);
    }

    void transact(uint8_t cmd, uint8_t writeNum, uint8_t readNum, uint8_t *writeBuf, uint8_t *readBuf){
        backEnd->i2c_transact(address, cmd, writeNum, readNum, writeBuf, readBuf);
    }

    void activate(){
        if (active()) { return; };

        try {
            uint8_t buffer[32];
            memcpy(buffer, POINTER(&(hardware.pwmMin)), 2);
            memcpy(buffer + 2, POINTER(&(hardware.pwmMax)), 2);
            memcpy(buffer + 4, POINTER(&(hardware.pwmPeriod)), 2);
            transact(CONFIG_PWM, buffer, nullptr);

            memcpy(buffer, POINTER(&(kP)), 4);
            memcpy(buffer + 4, POINTER(&(kI)), 4);
            memcpy(buffer + 8, POINTER(&(kD)), 4);
            transact(CONFIG_K, buffer, nullptr);

            uint16_t input = 0;
	    //transact(SPI, nullptr, POINTER(&input));

            int32_t angle = static_cast<int32_t>(quadCPR * ((static_cast<float>(input)/spiCPR) + (startAngle/(2.0*M_PI))));
            transact(ADJUST, POINTER(&angle), nullptr);
            
            transact(ON, nullptr, nullptr);

            backEnd->setHardwares(address, name);
        }
        catch (IOFailure &e) {
            printf("activate failed on %x\n", address);
            throw IOFailure();
        }
    }

    void recordAngle(int32_t angle) {

	float otherAngle = (static_cast<float>(angle)/quadCPR)*2.0*M_PI;
	if (std::abs(otherAngle - currentAngle) >= M_PI/16.0) {
		return;
	}
	currentAngle = otherAngle;
    }

public:
    Controller(uint8_t nucleo, uint8_t channel, Hardware inHardware) : hardware(inHardware){
        address = ((nucleo + 1) << 4) | channel;
    }

    void open_loop(float input) {
        std::scoped_lock controllerLock(controllerMutex);
        try {
            activate();

            uint16_t throttle = hardware.throttle(input);
            int32_t angle;
            transact(OPEN_PLUS, POINTER(&throttle), POINTER(&angle));

            recordAngle(angle);

        }
        catch (IOFailure &e) { printf("open loop failed on %x\n", address); }
    }

    void closed_loop(float torque, float angle) {
        std::scoped_lock controllerLock(controllerMutex);
        try {
            activate();

            float feedForward = 0;//torque * torqueScale;
            int32_t closedSetpoint = static_cast<int32_t>((angle / (2.0 * M_PI)) * quadCPR);
            uint8_t buffer[32]; int32_t angle;
            memcpy(buffer, POINTER(&feedForward), 4);
            memcpy(buffer+4, POINTER(&closedSetpoint), 4);
            transact(CLOSED_PLUS, buffer, POINTER(&angle));

            recordAngle(angle);
        }
        catch (IOFailure &e) { printf("closed loop failed on %x\n", address); }
    }

    void config(float KP, float KI, float KD){
        std::scoped_lock controllerLock(controllerMutex);
        
        for (int attempts = 0; attempts < 100; ++attempts) {
            try {
                activate();

                uint8_t buffer[32];
                memcpy(buffer, POINTER(&KP), 4);
                memcpy(buffer+4, POINTER(&KI), 4);
                memcpy(buffer+8, POINTER(&KD), 4);
                transact(CONFIG_K, buffer, nullptr);

                return;
            }
            catch (IOFailure &e) { printf("config failed on %x\n", address); }
        }
    }

    void zero(){
        std::scoped_lock controllerLock(controllerMutex);
        
        for (int attempts = 0; attempts < 100; ++attempts) {
            try {
                activate();

                int32_t zero = 0;
                transact(ADJUST, POINTER(&zero), nullptr);

                return;
            }
            catch (IOFailure &e) { printf("zero failed on %x\n", address); }
        }
    }

    void angle(){
        std::scoped_lock controllerLock(controllerMutex);
        if (!active()) {return;}

        try {
            int32_t angle;
            transact(QUAD, nullptr, POINTER(&angle));
            recordAngle(angle);
        }
        catch (IOFailure &e) { printf("angle failed on %x\n", address); }
    }

};

#endif
