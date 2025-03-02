#include "laserMapping.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions options;
        auto node = std::make_shared<rclcpp::Node>("fastlio_mapping", options);

        std::cout << "Starting FAST-LIO Mapping Node";

        // Initialize the laser mapping node
        auto laserMappingNode = std::make_shared<LaserMappingNode>(*node);
        std::cout << "FAST-LIO Mapping Node initialized" << std::endl;

        // Start spinning
        RCLCPP_INFO(node->get_logger(), "Starting spin");
        rclcpp::spin(node);

        // Cleanup
        if (rclcpp::ok())
        {
            RCLCPP_INFO(node->get_logger(), "Shutting down");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(node->get_logger(), "Node shutdown complete");
        return 0;
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("fastlio_mapping"),
                     "Exception in main: %s",
                     e.what());
        return 1;
    }
    catch (...)
    {
        RCLCPP_FATAL(rclcpp::get_logger("fastlio_mapping"),
                     "Unknown exception in main");
        return 1;
    }
}
