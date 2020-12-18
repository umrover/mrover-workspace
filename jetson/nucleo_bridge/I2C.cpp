#include "I2C.h"

//Abstraction for I2C/Hardware related functions
void I2C::init()
{
    file = open("/dev/i2c-1", O_RDWR);
    if (file == -1)
    {
        printf("failed to open i2c bus\n");
        exit(1);
    }
}

//Performs an i2c transaction
void I2C::transact(uint8_t addr, uint8_t cmd, uint8_t writeNum, uint8_t readNum, uint8_t *writeBuf, uint8_t *readBuf)
{
    if (file == -1)
    {
        printf("I2C Port never opened");
        throw IOFailure();
    }
    uint8_t buffer[32];

    buffer[0] = cmd;
    memcpy(buffer + 1, writeBuf, writeNum);

    ioctl(file, I2C_SLAVE, addr);

    if (writeNum + 1 != 0)
    {
        if (write(file, buffer, writeNum + 1) != writeNum + 1)
        {
            throw IOFailure();
        }
    }
    if (readNum != 0)
    {
        if (read(file, buffer, readNum) != readNum)
        {
            throw IOFailure();
        }
    }

    memcpy(readBuf, buffer, readNum);
}
