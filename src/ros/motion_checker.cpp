#include "ros/motion_checker.hpp"
#include "icp/geometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace ICP {

MotionChecker::MotionChecker(std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                             std::string odom_frame, 
                             std::string base_frame, 
                             double rate) : 
tf_buffer_(tf_buffer),
odom_frame_(odom_frame),
base_frame_(base_frame),
rate_(rate) {}

MotionChecker::~MotionChecker() {
    Stop();
    thread_handle_.join();
}

bool MotionChecker::GetCurrentBaseToOdomTransform(ROSTF& t) {
    try {
        t = tf_buffer_->lookupTransform(
        base_frame_,
        odom_frame_,
        tf2::TimePointZero);

        return true;
        
    } catch (const tf2::TransformException & ex) {
        return false;
    }
}

bool MotionChecker::Running() {
    return running_;
}

void MotionChecker::Start() {
    thread_handle_ = std::thread(&MotionChecker::Run, this);
}

bool MotionChecker::HasMovedBy(ROSTF curr_tf, ROSTF last_tf, float distance_m, float rotation_r) {
    double dx = curr_tf.transform.translation.x - last_tf.transform.translation.x;
    double dy = curr_tf.transform.translation.y - last_tf.transform.translation.y;
    double dz = curr_tf.transform.translation.z - last_tf.transform.translation.z;

    double d_pos_sq = (dx*dx + dy*dy + dz*dz);

    if (d_pos_sq > distance_m * distance_m) {
        return true;
    }

    double curr_roll, curr_pitch, curr_yaw;
    double last_roll, last_pitch, last_yaw;

    tf2::Quaternion curr_q;
    tf2::fromMsg(curr_tf.transform.rotation, curr_q);
    tf2::Matrix3x3 curr_m (curr_q);
    curr_m.getRPY(curr_roll, curr_pitch, curr_yaw);


    tf2::Quaternion last_q;
    tf2::fromMsg(last_tf.transform.rotation, last_q);
    tf2::Matrix3x3 last_m (last_q);
    last_m.getRPY(last_roll, last_pitch, last_yaw);

    double droll = Geometry::AbsoluteAngleDifference(curr_roll, last_roll);
    double dpitch = Geometry::AbsoluteAngleDifference(curr_pitch, last_pitch);
    double dyaw = Geometry::AbsoluteAngleDifference(curr_yaw, last_yaw);

    if (droll > rotation_r || dpitch > rotation_r || dyaw > rotation_r) {
        return true;
    }

    return false;
}

void MotionChecker::ResetTF() {
    std::lock_guard<std::mutex> l (tf_lock_);
    last_tf_ = curr_tf_;
}
bool MotionChecker::HasMovedBy(float distance_m, float rotation_r) {
    ROSTF curr_tf;
    ROSTF last_tf;

    {
        std::lock_guard<std::mutex>lock(tf_lock_);
        curr_tf = curr_tf_;
        last_tf = last_tf_;
    }

    return HasMovedBy(curr_tf, last_tf, distance_m, rotation_r);
}

void MotionChecker::Run() {

    while (should_run_) {
        
        if (!GetCurrentBaseToOdomTransform(curr_tf_)) {
            std::cout <<"problem getting tf" << std::endl;
        }

        rate_.sleep();
    }
    
    running_ = false;
}

void MotionChecker::Stop() {
    should_run_ = false;
}

} // namespace ICP
