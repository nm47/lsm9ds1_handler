#include "lsm9ds1_handler/lsm9ds1.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto imu = std::make_shared<lsm9ds1::LSM9DS1>("lsm9ds1");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imu->node_);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
