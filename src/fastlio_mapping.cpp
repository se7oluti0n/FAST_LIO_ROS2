#include "laserMapping.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // signal(SIGINT, Sigandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();

    return 0;
}
