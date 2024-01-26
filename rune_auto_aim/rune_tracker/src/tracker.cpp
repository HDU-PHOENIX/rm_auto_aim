#include "rune_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rune {
Tracker::Tracker(double&& std_a_, double&& std_yawdd_) {
    ukf = new UKF_PLUS(false, true, false, std_a_, std_yawdd_);
}
} // namespace rune
