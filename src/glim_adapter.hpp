#pragma once
#include <atomic>
#include <thread>
#include <functional>
#include <vector>
#include <memory>


#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

class LaserMappingNode;

namespace glim
{
    class GlimAdapter: public ExtensionModuleROS2
    {
    public:
        GlimAdapter() {}
        ~GlimAdapter() {}
        virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node &node) override;

    private:
        // void set_callbacks();
        // void invoke(const std::function<void()>& task);
        //
        void spin_once();

    private:
        std::atomic_bool kill_switch;
        std::thread thread;


        std::shared_ptr<LaserMappingNode> laser_mapping_node_;
    };

} // namespace glim
