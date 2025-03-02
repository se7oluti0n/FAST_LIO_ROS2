#include "glim_adapter.hpp"
#include "laserMapping.hpp"

namespace glim
{
    std::vector<GenericTopicSubscription::Ptr> GlimAdapter::create_subscriptions(rclcpp::Node &node)
    {
        std::vector<GenericTopicSubscription::Ptr> result;

        laser_mapping_node_ = std::make_shared<LaserMappingNode>(node);
        return result;
    }


    void GlimAdapter::spin_once()
    {
      // const auto& frame = raw_frames.front();
      // std::vector<EstimationFrame::ConstPtr> marginalized;
      // auto state = odometry_estimation->insert_frame(frame, marginalized);
      //
      // output_estimation_results.push_back(state);
      // output_marginalized_frames.insert(marginalized);
      // raw_frames.pop_front();
      // internal_frame_queue_size = raw_frames.size();

      // make lasermapping node a spin_once and output_marginalized_frames (EstimationFrame::ConstPtr)

    }

} // namespace glim
