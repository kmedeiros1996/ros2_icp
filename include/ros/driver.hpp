#pragma once

/*
* Scanmatch driver code header file
* Author - Skye Medeiros
*/
#include <rclcpp/rclcpp.hpp>

#include "icp/types.hpp"
#include "ros/motion_checker.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/node_options.hpp"

namespace ICP {

class ScanMatchDriver: public nav2_util::LifecycleNode { 
public:
~ScanMatchDriver();

explicit ScanMatchDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

rcl_interfaces::msg::SetParametersResult DynamicParametersCallback(ParametersVector parameters);

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  
void InitializeParameters();
void InitializeTF();
void InitializeMessageFilters();
void InitializePublishers();

void OnReceiveLaserScan(const ROSLaserScan::ConstSharedPtr scan_msg);

private:
bool                                                     initialized_{false};
std::shared_ptr<tf2_ros::TransformListener>              tf_listener_;
std::shared_ptr<tf2_ros::Buffer>                         tf_buffer_;

std::unique_ptr<MotionChecker>                           motion_checker_;

rclcpp::CallbackGroup::SharedPtr                         callback_group_;
rclcpp::executors::SingleThreadedExecutor::SharedPtr     executor_;
std::unique_ptr<nav2_util::NodeThread>                   executor_thread_;

std::unique_ptr<LaserScanSubscriber>                    laser_scan_sub_;
std::unique_ptr<LaserScanFilter>                        laser_scan_filter_;
message_filters::Connection                             laser_scan_connection_;
rclcpp::Time                                            last_scan_match_time_;


tf2::Duration                                           transform_tolerance_;
std::string                                             scan_topic_{"scan"};
std::string                                             odom_frame_id_{"odom"};
std::string                                             base_frame_id_{"base_link"};
double                                                  scan_match_min_rotation_{0.25};
double                                                  scan_match_min_translation_{0.1};
};
} // namespace ICP