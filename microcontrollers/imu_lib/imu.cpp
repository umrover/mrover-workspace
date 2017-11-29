#include "imu.hpp"

Imu::Imu(PinName sda, PinName scl) :
    i2c_(sda, scl),
    valid_(false),
    check_valid_(false)
{
    init();
}

void Imu::init() {
    memset(&this->accel_, 0, sizeof(this->accel_));
    memset(&this->mag_, 0, sizeof(this->mag_));
    memset(&this->gyro_, 0, sizeof(this->gyro_));

    this->check_valid_ = false;
    // -- Initialize Accelerometer/Magnetometer --
    // Ensure that it's properly connected
    uint8_t id = this->read_am_reg(AM_REGISTER_WHO_AM_I);
    if (id != AM_ID) {
        this->check_valid_ = true;
        return; // valid_ = false
    }

    // Set the IMU to standby
    this->write_am_reg(AM_REGISTER_CTRL_REG1, 0);

    // Configure accelerometer
    // TODO make the range customizable (rn hard-coded for 4G)
    this->write_am_reg(AM_REGISTER_XYZ_DATA_CFG, 0x01);
    // Enable high-resolution mode
    this->write_am_reg(AM_REGISTER_CTRL_REG2, 0x02);
    // Active, Normal, Low-noise, 100 Hz, hybrid
    this->write_am_reg(AM_REGISTER_CTRL_REG1, 0x15);

    // Configure magnetometer
    // Hybrid, oversampling rate=16
    this->write_am_reg(AM_REGISTER_MCTRL_REG1, 0x1F);
    // Jump to reg 0x33 after reading 0x06
    this->write_am_reg(AM_REGISTER_MCTRL_REG2, 0x20);

    // -- Initialize Gyroscope --
    // Ensure that it's properly connected
    id = this->read_gy_reg(GY_REGISTER_WHO_AM_I);
    if (id != GY_ID) {
        this->check_valid_ = true;
        return; // valid_ = false
    }

    // Set standby
    this->write_gy_reg(GY_REGISTER_CTRL_REG1, 0x00);
    // Reset gyro
    this->write_gy_reg(GY_REGISTER_CTRL_REG1, (1<<6));
    // Set sensitivity to 1000 dps TODO configurable
    this->write_gy_reg(GY_REGISTER_CTRL_REG1, 0x01);
    // Set active
    this->write_gy_reg(GY_REGISTER_CTRL_REG1, 0x0E);
    wait(0.1);

    this->valid_ = true;
    this->check_valid_ = true;
}

bool Imu::valid() const {
    return this->valid_;
}

void Imu::read() {
    memset(&this->accel_, 0, sizeof(this->accel_));
    memset(&this->mag_, 0, sizeof(this->mag_));
    memset(&this->gyro_, 0, sizeof(this->gyro_));

    char msg = AM_REGISTER_STATUS | 0x80;
    this->i2c_.write(AM_ADDRESS, &msg, 1);

    /*
     *  [0] = status
     *  [1] = Accelerometer X high byte
     *  [2] = Accelerometer X low byte
     *  [3] = Accelerometer Y high byte
     *  [4] = Accelerometer Y low byte
     *  [5] = Accelerometer Z high byte
     *  [6] = Accelerometer Z low byte
     *  [7] = Magnetometer X high byte
     *  [8] = Magnetometer X low byte
     *  [9] = Magnetometer Y high byte
     * [10] = Magnetometer Y low byte
     * [11] = Magnetometer Z high byte
     * [12] = Magnetometer Z low byte
     */
    char am_recv_buf[13];
    this->i2c_.read(AM_ADDRESS, am_recv_buf, 13);

    msg = GY_REGISTER_STATUS | 0x80;
    this->i2c_.write(GY_ADDRESS, &msg, 1);

    /*
     * [0] = status
     * [1] = Gyroscope X high byte
     * [2] = Gyroscope X low byte
     * [3] = Gyroscope Y high byte
     * [4] = Gyroscope Y low byte
     * [5] = Gyroscope Z high byte
     * [6] = Gyroscope Z low byte
     */
    char gy_recv_buf[7];
    this->i2c_.read(GY_ADDRESS, gy_recv_buf, 7);

    int16_t accel_x = (int16_t)((am_recv_buf[1] << 8) | am_recv_buf[2]) >> 2;
    int16_t accel_y = (int16_t)((am_recv_buf[3] << 8) | am_recv_buf[4]) >> 2;
    int16_t accel_z = (int16_t)((am_recv_buf[5] << 8) | am_recv_buf[6]) >> 2;

    int16_t mag_x = (int16_t)((am_recv_buf[7] << 8) | am_recv_buf[8]);
    int16_t mag_y = (int16_t)((am_recv_buf[9] << 8) | am_recv_buf[10]);
    int16_t mag_z = (int16_t)((am_recv_buf[11] << 8) | am_recv_buf[12]);

    int16_t gyro_x = (int16_t)((gy_recv_buf[1] << 8) | gy_recv_buf[2]);
    int16_t gyro_y = (int16_t)((gy_recv_buf[3] << 8) | gy_recv_buf[4]);
    int16_t gyro_z = (int16_t)((gy_recv_buf[5] << 8) | gy_recv_buf[6]);

    this->accel_.x = accel_x * ACCEL_MG_LSB_4G * GRAVITY_STANDARD;
    this->accel_.y = accel_y * ACCEL_MG_LSB_4G * GRAVITY_STANDARD;
    this->accel_.z = accel_z * ACCEL_MG_LSB_4G * GRAVITY_STANDARD;

    this->mag_.x = mag_x * MAG_UT_LSB;
    this->mag_.y = mag_y * MAG_UT_LSB;
    this->mag_.z = mag_z * MAG_UT_LSB;

    this->gyro_.x = gyro_x * GYRO_SENSITIVITY_1000DPS * DPS_TO_RADS;
    this->gyro_.y = gyro_y * GYRO_SENSITIVITY_1000DPS * DPS_TO_RADS;
    this->gyro_.z = gyro_z * GYRO_SENSITIVITY_1000DPS * DPS_TO_RADS;
}

Math::Vector3f Imu::accelerometer() const {
    return this->accel_;
}

Math::Vector3f Imu::magnetometer() const {
    return this->mag_;
}

Math::Vector3f Imu::gyroscope() const {
    return this->gyro_;
}

void Imu::write_am_reg(uint8_t reg, uint8_t value) {
    this->write_reg(AM_ADDRESS, reg, value);
}

uint8_t Imu::read_am_reg(uint8_t reg) {
    return this->read_reg(AM_ADDRESS, reg);
}

void Imu::write_gy_reg(uint8_t reg, uint8_t value) {
    this->write_reg(GY_ADDRESS, reg, value);
}

uint8_t Imu::read_gy_reg(uint8_t reg) {
    return this->read_reg(GY_ADDRESS, reg);
}

void Imu::write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    if (this->check_valid_ && !this->valid_) return;
    char buf[2] = {reg, value};
    if (this->i2c_.write(addr, buf, 2) != 0) {
        this->valid_ = false;
    }
}

uint8_t Imu::read_reg(uint8_t addr, uint8_t reg) {
    if (this->check_valid_ && !this->valid_) return 255;
    char signed_reg = (char) reg;
    this->i2c_.write(addr, &signed_reg, 1, true);
    char value;
    if (this->i2c_.read(addr, &value, 1) != 0) {
        this->valid_ = false;
    }
    return (uint8_t) value;
}
