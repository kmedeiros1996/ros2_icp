#include "tf2_ros/buffer.h"
#include "icp/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <atomic>

namespace ICP {
class MotionChecker {
public:
    MotionChecker(std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                  std::string odom_frame, 
                  std::string base_frame, 
                  double rate);
    ~MotionChecker();

    bool HasMovedBy(float distance_m, float rotation_r);
    bool HasMovedBy(ROSTF curr_tf, ROSTF last_tf, float distance_m, float rotation_r);
    bool GetCurrentBaseToOdomTransform(ROSTF& t);

    void ResetTF();
    void Start();
    bool Running();
    void Run();
    void Stop();

private:
    ROSTF curr_tf_;
    ROSTF last_tf_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Rate rate_;
    std::string odom_frame_;
    std::string base_frame_;

    std::thread thread_handle_;
    std::atomic<bool> running_{false};
    std::atomic<bool> should_run_{true};
    std::mutex tf_lock_;
};
} // namespace ICP