#include "lsm9ds1_handler/lsm9ds1_handler.hpp"

namespace lsm9ds1
{
LSM9DS1::LSM9DS1(const std::string &name_) : imu_name_(name_)
{

    declare_ROS_params();
    initialize();
}

LSM9DS1::~LSM9DS1()
{
    node_.reset();
    lsm9ds1_device_.reset();
}

void LSM9DS1::declare_ROS_params()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initializing lsm9ds1 handler");

    node_ = std::make_shared<rclcpp::Node>(imu_name_);
    node_->declare_parameter<uint8_t>("i2c_interface", 1);
    node_->declare_parameter<uint8_t>("i2c_address_mag", 0x1e);
    node_->declare_parameter<uint8_t>("i2c_address_accelgyro", 0x6b);

    node_->declare_parameter<uint8_t>("accel_rate", 0);
    node_->declare_parameter<uint8_t>("accel_scale", 0);

    node_->declare_parameter<uint8_t>("gyro_rate", 0);
    node_->declare_parameter<uint8_t>("gyro_scale", 0);

    node_->declare_parameter<uint8_t>("mag_rate", 0);
    node_->declare_parameter<uint8_t>("mag_scale", 0);
}

void LSM9DS1::initialize()
{
    uint8_t bus_index = node_->get_parameter("i2c_interface").get_parameter_value().get<uint8_t>();

    uint8_t i2c_address_accelgyro = node_->get_parameter("i2c_address_accelgyro").get_parameter_value().get<uint8_t>();

    uint8_t i2c_address_mag = node_->get_parameter("i2c_address_mag").get_parameter_value().get<uint8_t>();

    uint8_t accel_rate = node_->get_parameter("accel_rate").get_parameter_value().get<uint8_t>();

    uint8_t accel_scale = node_->get_parameter("accel_scale").get_parameter_value().get<uint8_t>();

    uint8_t gyro_rate = node_->get_parameter("gyro_rate").get_parameter_value().get<uint8_t>();

    uint8_t gyro_scale = node_->get_parameter("gyro_scale").get_parameter_value().get<uint8_t>();

    uint8_t mag_rate = node_->get_parameter("mag_rate").get_parameter_value().get<uint8_t>();
    uint8_t mag_scale = node_->get_parameter("mag_scale").get_parameter_value().get<uint8_t>();

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "bus_index_ %d", bus_index);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "i2c_address_mag %d", i2c_address_mag);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "i2c_address_accelgyro %d", i2c_address_accelgyro);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "accel_rate %d", accel_rate);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "accel_scale %d", accel_scale);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "gyro_rate %d", gyro_rate);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "gyro_scale %d", gyro_scale);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "mag_rate %d", mag_rate);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "mag_scale %d", mag_scale);

    lsm9ds1_device_ = std::make_shared<LSM9DS1_Device>(bus_index, i2c_address_accelgyro, i2c_address_mag);
    lsm9ds1_device_->configure_accel(accel_scale, accel_rate);
    lsm9ds1_device_->configure_gyro(gyro_scale, gyro_rate);
    lsm9ds1_device_->configure_mag(mag_scale, mag_rate, true);

    lsm9ds1_device_->calibrate_accelgyro();

    publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(imu_name_ + "/telemetry", 10);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&LSM9DS1::read_IMU, this));
}

void LSM9DS1::read_IMU()
{
    IMURecord imu_record = lsm9ds1_device_->read_all();

    // telemetry_msg_.magnetic_field.x = imu_record.raw_magnetic_field.x;
    // telemetry_msg_.magnetic_field.y = imu_record.raw_magnetic_field.y;
    // telemetry_msg_.magnetic_field.z = imu_record.raw_magnetic_field.z;

    telemetry_msg_.linear_acceleration.x = imu_record.raw_linear_acceleration.x;
    telemetry_msg_.linear_acceleration.y = imu_record.raw_linear_acceleration.y;
    telemetry_msg_.linear_acceleration.z = imu_record.raw_linear_acceleration.z;

    telemetry_msg_.angular_velocity.x = imu_record.raw_angular_velocity.x;
    telemetry_msg_.angular_velocity.y = imu_record.raw_angular_velocity.y;
    telemetry_msg_.angular_velocity.z = imu_record.raw_angular_velocity.z;

    publish();
}

void LSM9DS1::publish()
{
    publisher_->publish(telemetry_msg_);
}
} // namespace lsm9ds1
