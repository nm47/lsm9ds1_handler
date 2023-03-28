#include "lsm9ds1_handler/lsm9ds1_device.hpp"

namespace lsm9ds1
{
LSM9DS1_Device::LSM9DS1_Device(const int bus_index, const uint8_t i2c_address_accelgyro, const uint8_t i2c_address_mag)
    : bus_index_(bus_index), i2c_address_accelgyro_(i2c_address_accelgyro), i2c_address_mag_(i2c_address_mag)
{
    initialize();
}

LSM9DS1_Device::~LSM9DS1_Device()
{
    i2c_smbus_write_byte_data(fd_accelgyro_, CTRL_REG1_G, 0);
    i2c_smbus_write_byte_data(fd_accelgyro_, CTRL_REG6_XL, 0);
    // i2c_smbus_write_byte_data(fd_mag_, CTRL_REG3_M, 0);

    close(fd_accelgyro_);
    close(fd_mag_);
}

void LSM9DS1_Device::configure_gyro(const uint8_t scale, const uint8_t rate)
{
    gyro_scale_ = scale;
    gyro_rate_ = rate;

    // wake up gyro, set rate and scale
    uint8_t gyro_config = (gyro_rate_ << 5) | (gyro_scale_ << 3);
    i2c_smbus_write_byte_data(fd_accelgyro_, CTRL_REG1_G, gyro_config);
}

void LSM9DS1_Device::configure_mag(const uint8_t scale, const uint8_t rate, bool temp_comp)
{
    mag_scale_ = scale;
    mag_rate_ = rate;

    // wake up mag, set rate and scale
    uint8_t mag_reg1 = (temp_comp << 7) | (static_cast<uint8_t>(settings::kMagPerformance::kMagPerformanceUltra) << 5) | (mag_rate_ << 2);
    uint8_t mag_reg2 = (mag_scale_ << 5);

    i2c_smbus_write_byte_data(fd_mag_, CTRL_REG1_M, mag_reg1);
    i2c_smbus_write_byte_data(fd_mag_, CTRL_REG2_M, mag_reg2);
    i2c_smbus_write_byte_data(fd_mag_, CTRL_REG3_M, 0);
}

void LSM9DS1_Device::configure_accel(const uint8_t scale, const uint8_t rate)
{
    accel_scale_ = scale;
    accel_rate_ = rate;

    // wake up accelerometer, set rate and scale
    uint8_t accel_config = (accel_rate_ << 5) | (accel_scale_ << 3);
    i2c_smbus_write_byte_data(fd_accelgyro_, CTRL_REG6_XL, accel_config);
}

void LSM9DS1_Device::initialize()
{
    std::string device_path = "/dev/i2c-" + std::to_string(bus_index_);

    fd_accelgyro_ = open(device_path.c_str(), O_RDWR);
    fd_mag_ = open(device_path.c_str(), O_RDWR);

    if ((fd_accelgyro_ < 0) | (fd_mag_ < 0))
    {
        int errsv = errno; // printf may overwrite errno, save ioctl result
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ioctl failed to open device at %s: %s", device_path.c_str(),
                     strerror(errsv));
    }

    // no need to mutex reads/writes, linux i2c driver handles this for us.
    if (ioctl(fd_accelgyro_, I2C_SLAVE, i2c_address_accelgyro_) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find accel/gyro at address: %d.", i2c_address_accelgyro_);
        exit(1);
    }

    if (ioctl(fd_mag_, I2C_SLAVE, i2c_address_mag_) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find mag address: %d.", i2c_address_mag_);
        exit(1);
    }

    if (!check_devices())
        exit(1);
}

bool LSM9DS1_Device::check_devices()
{
    if (i2c_smbus_read_byte_data(fd_accelgyro_, WHO_AM_I_AG) == WHO_AM_I_AG_RSP)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "found Accel/Gyro at address %#2x", i2c_address_accelgyro_);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to access accelgyro: %s", strerror(errno));
        return false;
    }

    if (i2c_smbus_read_byte_data(fd_mag_, WHO_AM_I_M) == WHO_AM_I_M_RSP)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "found Magnetometer at address %#2x", i2c_address_mag_);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to access Magnetometer: %s", strerror(errno));
        return false;
    }
    return true;
}

void LSM9DS1_Device::calibrate_accelgyro()
{
}

void LSM9DS1_Device::read_accel(Axis3 &accels, bool scaled/* = true*/)
{
    // read 6 bytes, LSM9DS1 will automatically increment the register
    // so we can read all the raw accel/gyro data in one go.
    uint8_t raw_accel[6];
    i2c_smbus_read_i2c_block_data(fd_accelgyro_, OUT_X_L_XL, 6, raw_accel);

    // combine H/L 8 bit vals into a single 16 bit int, convert from twos complement
    int16_t raw_linear_accel_x = raw_accel[0] | (raw_accel[1] << 8);
    int16_t raw_linear_accel_y = raw_accel[2] | (raw_accel[3] << 8);
    int16_t raw_linear_accel_z = raw_accel[4] | (raw_accel[5] << 8);

    accels.x = (float)raw_linear_accel_x;
    accels.y = (float)raw_linear_accel_y;
    accels.z = (float)raw_linear_accel_z;

    // multiply by accelerometer sensitivity values from datasheet and convert to m/s^2
    if (scaled){
        accels.x = (accels.x - accel_offset_.x) * settings::kAccelSensMap.at(accel_scale_);
        accels.x = (accels.x / 1000.0f) * constants::kGravity;

        accels.y = (accels.y - accel_offset_.y) * settings::kAccelSensMap.at(accel_scale_);
        accels.y = (accels.y / 1000.0f) * constants::kGravity;

        accels.z = (accels.z - accel_offset_.z) * settings::kAccelSensMap.at(accel_scale_);
        accels.z = (accels.z / 1000.0f) * constants::kGravity;
    }
}

void LSM9DS1_Device::read_gyro(Axis3 &angular_vel, bool scaled/* = true*/)
{
    // read 12 bytes, LSM9DS1 will automatically increment the register
    // so we can read all the raw accel/gyro data in one go.
    uint8_t raw_vel[6];
    i2c_smbus_read_i2c_block_data(fd_accelgyro_, OUT_X_L_G, 6, raw_vel);

    // combine H/L 8 bit vals into a single 16 bit int, convert from twos complement
    int16_t raw_angular_velocity_x = raw_vel[0] | (raw_vel[1] << 8);
    int16_t raw_angular_velocity_y = raw_vel[2] | (raw_vel[3] << 8);
    int16_t raw_angular_velocity_z = raw_vel[4] | (raw_vel[5] << 8);

    angular_vel.x = (float)raw_angular_velocity_x;
    angular_vel.y = (float)raw_angular_velocity_y;
    angular_vel.z = (float)raw_angular_velocity_z;

    // multiply by gyro sensitivity values from datasheet and convert to dps
    if (scaled){
        angular_vel.x =
            ((angular_vel.x) * settings::kGyroSensMap.at(gyro_scale_)) / 1000.0f;
        angular_vel.y =
            ((angular_vel.y) * settings::kGyroSensMap.at(gyro_scale_)) / 1000.0f;
        angular_vel.z =
            ((angular_vel.z) * settings::kGyroSensMap.at(gyro_scale_)) / 1000.0f;
    }
}

void LSM9DS1_Device::read_mag(Axis3 &mag_gauss, bool scaled/* = true*/)
{
    // read 12 bytes, LSM9DS1 will automatically increment the register
    // so we can read all the raw accel/gyro data in one go.
    uint8_t mag_vals[6];
    i2c_smbus_read_i2c_block_data(fd_mag_, OUT_X_L_M, 6, mag_vals);

    // combine H/L 8 bit vals into a single 16 bit int, convert from twos complement
    int16_t raw_gauss_x = mag_vals[0] | (mag_vals[1] << 8);
    int16_t raw_gauss_y = mag_vals[2] | (mag_vals[3] << 8);
    int16_t raw_gauss_z = mag_vals[4] | (mag_vals[5] << 8);

    mag_gauss.x = (float)raw_gauss_x;
    mag_gauss.y = (float)raw_gauss_y;
    mag_gauss.z = (float)raw_gauss_z;

    // multiply by gyro sensitivity values from datasheet and convert to dps
    if (scaled){
        mag_gauss.x =
            ((mag_gauss.x) * settings::kMagSensMap.at(mag_scale_)) / 1000.0f;
        mag_gauss.y =
            ((mag_gauss.y) * settings::kMagSensMap.at(mag_scale_)) / 1000.0f;
        mag_gauss.z =
            ((mag_gauss.z) * settings::kMagSensMap.at(mag_scale_)) / 1000.0f;
    }
}

IMURecord LSM9DS1_Device::read_all(bool scaled /*= true*/){
    IMURecord imu_data;
    
    read_accel(imu_data.raw_linear_acceleration, scaled);
    read_gyro(imu_data.raw_angular_velocity, scaled);
    read_mag(imu_data.raw_magnetic_field, scaled);

    return imu_data;
}
} // namespace lsm9ds1
