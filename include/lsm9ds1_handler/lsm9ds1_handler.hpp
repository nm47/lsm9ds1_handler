#ifndef LSM9DS1_HPP

#include "lsm9ds1_device.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>

namespace lsm9ds1
{
class LSM9DS1
{
  public:
    ~LSM9DS1();
    LSM9DS1(const std::string &name_);
    rclcpp::Node::SharedPtr node_;

  protected:
    void declare_ROS_params();
    void configure();
    void initialize();
    void read_IMU();
    void publish();

  private:
    std::string imu_name_;
    sensor_msgs::msg::Imu telemetry_msg_;
    std::shared_ptr<LSM9DS1_Device> lsm9ds1_device_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace lsm9ds1

#endif
