#include "ros/driver.hpp"
#include "icp/types.hpp"
#include "rclcpp/node_options.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace ICP {

ScanMatchDriver::ScanMatchDriver(const rclcpp::NodeOptions & options)
                                : nav2_util::LifecycleNode("icp_driver", "", options){

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  add_parameter(
    "odom_frame_id", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for odometry");

  add_parameter(
    "base_frame_id", rclcpp::ParameterValue(std::string("base_link")),
    "Which frame to use as the base link frame");

  add_parameter(
    "scan_topic", rclcpp::ParameterValue("scan"),
    "Topic to subscribe to in order to receive the laser scan for scan matching");

  add_parameter(
    "scan_match_min_rotation", rclcpp::ParameterValue(0.2),
    "Rotational movement required before running scan registration");

  add_parameter(
    "scan_match_min_translation", rclcpp::ParameterValue(0.25),
    "Translational movement required before running scan registration");

  add_parameter(
    "transform_tolerance", rclcpp::ParameterValue(1.0),
    "tolerance when looking up transforms");
}

nav2_util::CallbackReturn ScanMatchDriver::on_configure(const rclcpp_lifecycle::State& state) {
  InitializeTF();
  InitializeMessageFilters();
  InitializeParameters();
  InitializePublishers();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScanMatchDriver::on_activate(const rclcpp_lifecycle::State& state) {

  RCLCPP_INFO(get_logger(), "Activating");

  motion_checker_ = std::make_unique<MotionChecker>(tf_buffer_, "odom", "base_link", 50);
  motion_checker_->Start();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ScanMatchDriver::DynamicParametersCallback,
      this, std::placeholders::_1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;

}

nav2_util::CallbackReturn ScanMatchDriver::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  // shutdown and reset dynamic parameter handler
  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  motion_checker_->Stop();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScanMatchDriver::on_cleanup(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Cleaning up!");

  laser_scan_connection_.disconnect();
  executor_thread_.reset();
  motion_checker_->Stop();


  return nav2_util::CallbackReturn::SUCCESS;

}

nav2_util::CallbackReturn ScanMatchDriver::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Shutting down ICP node!");
  return nav2_util::CallbackReturn::SUCCESS;
}


rcl_interfaces::msg::SetParametersResult 
ScanMatchDriver::DynamicParametersCallback(ParametersVector parameters) {
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const std::string& param_name = parameter.get_name();

    if (param_name == "odom_frame_id" || param_name == "base_frame_id" || param_name == "scan_topic") {
      RCLCPP_WARN(get_logger(), "Changing %s is not allowed.", param_name.c_str());
    }
    else if(param_name == "scan_match_min_rotation") {
      scan_match_min_rotation_ = parameter.as_double();
    }
    else if(param_name ==  "scan_match_min_translation") {
      scan_match_min_translation_ = parameter.as_double();
    }
  }
  
  result.successful = true;
  return result;

}

void ScanMatchDriver::InitializeParameters() {
  RCLCPP_INFO(get_logger(), "Initializing parameters!");

  double temp_transform_tolerance;


  get_parameter("transform_tolerance", temp_transform_tolerance);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("scan_match_min_rotation", scan_match_min_rotation_);
  get_parameter("scan_match_min_translation", scan_match_min_translation_);

  transform_tolerance_ = tf2::durationFromSec(temp_transform_tolerance);
}

void ScanMatchDriver::InitializeTF() {

  RCLCPP_INFO(get_logger(), "Initializing TF!");
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface(),
      callback_group_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ScanMatchDriver::InitializeMessageFilters() {
  RCLCPP_INFO(get_logger(), "Initializing message filters!");

  auto opts = rclcpp::SubscriptionOptions();
  opts.callback_group = callback_group_;

  laser_scan_sub_ = std::make_unique<LaserScanSubscriber>(shared_from_this(), 
                                                          scan_topic_, 
                                                          rmw_qos_profile_sensor_data, 
                                                          opts);

  laser_scan_filter_ = std::make_unique<LaserScanFilter>(
                      *laser_scan_sub_, 
                      *tf_buffer_, 
                      odom_frame_id_, 
                      100,
                      get_node_logging_interface(),
                      get_node_clock_interface(),
                      transform_tolerance_);


RCLCPP_WARN(get_logger(), "registering callback");

laser_scan_connection_ = laser_scan_filter_->registerCallback(
                                              std::bind(
                                              &ScanMatchDriver::OnReceiveLaserScan,
                                              this, 
                                              std::placeholders::_1)
                                            );
}

void ScanMatchDriver::InitializePublishers() {
  RCLCPP_INFO(get_logger(), "Initializing publishers!");

}

void ScanMatchDriver::OnReceiveLaserScan(const ROSLaserScan::ConstSharedPtr scan) {
  if (motion_checker_->HasMovedBy(scan_match_min_translation_, scan_match_min_rotation_)) {
    RCLCPP_INFO(get_logger(), "moved");
    motion_checker_->ResetTF();
  }
}

ScanMatchDriver::~ScanMatchDriver() {}

} // namespace ICP


#include "ros/driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"


RCLCPP_COMPONENTS_REGISTER_NODE(ICP::ScanMatchDriver)