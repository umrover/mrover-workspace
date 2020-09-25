#ifndef BACKEND_H
#define BACKEND_H

#include <string>
#include <unordered_map>
#include <mutex>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <exception>

struct IOFailure : public std::exception {};

class BackEnd {
private:
	int file;
	std::mutex i2cMutex;
	std::mutex hardwaresMutex;
	std::unordered_map<uint8_t, std::string> hardwares;

public:
	BackEnd(){
		file = open("/dev/i2c-1", O_RDWR);
		if (file == -1){ printf("failed to open i2c bus"); /*exit(1);*/ }
	}

	std::string getHardwares(uint8_t address) {
		return hardwares[address];
	}

	void setHardwares(uint8_t address, std::string value) {
		std::scoped_lock hardwaresLock(hardwaresMutex);
		hardwares[address] = value;
	}

	void i2c_transact(uint8_t addr, uint8_t cmd, uint8_t writeNum, uint8_t readNum, uint8_t *writeBuf, uint8_t *readBuf ){
		std::scoped_lock i2cLock(i2cMutex);
		uint8_t buffer[32];
		
		buffer[0] = cmd;
		memcpy(buffer + 1, writeBuf, writeNum);

		ioctl(file, I2C_SLAVE, addr);
		
		if (writeNum + 1 != 0) {
			if ( write(file, buffer, writeNum + 1) != writeNum + 1) { throw IOFailure();}
		}
		if (readNum != 0) {
			if ( read(file, buffer, readNum) != readNum ) { throw IOFailure(); }
		}

		memcpy(readBuf, buffer, readNum);
	}
};

#endif
