#ifndef LSM9DS1_DEVICE_HPP

extern "C"
{
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <iostream>
#include <lsm9ds1_handler/lsm9ds1_registers.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lsm9ds1
{

namespace constants
{
static constexpr float kGravity = 9.81;
}

namespace settings
{

// ask STMicroElectronics why 16 comes before 4..
enum kAccelScale
{
    kAccelScale2,
    kAccelScale16,
    kAccelScale4,
    kAccelScale8
};
enum kAccelRate
{
    kAccelPowerDown,
    kAccelHz10,
    kAccelHz50,
    kAccelHz119,
    kAccelHz238,
    kAccelHz476,
    kAccelHz952
};

enum kGyroScale
{
    kGyroScale245,
    kGyroScale500,
    kUnavailable,
    kGyroScale2000
};
enum kGyroRate
{
    kGyroPowerDown,
    kGyroHz14_9,
    kGyroHz59_5,
    kGyroHz119,
    kGyroHz238,
    kGyroHz476,
    kGyroHz952
};

enum kMagScale
{
    kMagScale4,
    kMagScale8,
    kMagScale12,
    kMagScale16
};
enum kMagRate
{
    kMagHz0_625,
    kMagHz1_25,
    kMagHz2_5,
    kMagHz5,
    kMagHz10,
    kMagHz20,
    kMagHz40,
    kMagHz80
};
enum kMagPerformance
{
    kMagPerformanceLowPower,
    kMagPerformanceMedium,
    kMagPerformanceHigh,
    kMagPerformanceUltra
};

const std::unordered_map<uint8_t, float> kAccelSensMap{
    {kAccelScale2, 0.061f}, {kAccelScale4, 0.122f}, {kAccelScale8, 0.244f}, {kAccelScale16, 0.732f}};
const std::unordered_map<uint8_t, float> kGyroSensMap{
    {kGyroScale245, 0.14f}, {kGyroScale500, 17.50f}, {kGyroScale2000, 70.0f}};
const std::unordered_map<uint8_t, float> kMagSensMap{
    {kMagScale4, 0.14f}, {kMagScale8, 0.29f}, {kMagScale12, 0.43f}, {kMagScale16, 0.58f}};
} // namespace settings

typedef struct
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
} Axis3;

typedef struct
{
    Axis3 raw_linear_acceleration;
    Axis3 raw_angular_velocity;
    Axis3 raw_magnetic_field;
} IMURecord;

class LSM9DS1_Device
{
  public:
    ~LSM9DS1_Device();
    LSM9DS1_Device(const int bus_index, const uint8_t i2c_address_accelgyro, const uint8_t i2c_address_mag);

    void read_accel(Axis3 &accels, bool scaled = true);
    void read_gyro(Axis3 &angular_vel, bool scaled = true);
    void read_mag(Axis3 &mag_gauss, bool scaled = true);
    IMURecord read_all(bool scaled = true);

    void calibrate_accelgyro();
    void configure_accel(const uint8_t scale, const uint8_t rate);
    void configure_gyro(const uint8_t scale, const uint8_t rate);
    void configure_mag(const uint8_t scale, const uint8_t rate, bool temp_comp = true);

  protected:
    void initialize();
    void configure();
    bool reset();
    bool check_devices();

  private:
    int bus_index_;
    int fd_accelgyro_;
    int fd_mag_;
    int calibration_samples_ = 60;

    uint8_t i2c_address_accelgyro_;
    uint8_t accel_scale_;
    uint8_t accel_rate_;
    Axis3 accel_offset_;

    uint8_t gyro_scale_;
    uint8_t gyro_rate_;

    uint8_t i2c_address_mag_;
    uint8_t mag_scale_;
    uint8_t mag_rate_;
};
} // namespace lsm9ds1

#endif
