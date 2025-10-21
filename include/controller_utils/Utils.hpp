#ifndef CONTROLLER_UTILS___UTILS_HPP_
#define CONTROLLER_UTILS___UTILS_HPP_

#include <memory>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

namespace controller_utils::utils {
using namespace std::chrono_literals;  // NOLINT
/**
 * @brief Extract the interface type substring from a full interface name.
 *
 * If the input contains slashes, the last path component is returned. If no
 * slash exists, the input is returned unchanged.
 */
std::string getInterfaceTypeFromFullName(const std::string& fullInterfaceName);

/**
 * @brief Extract the upstream configuration segment from a full interface name.
 *
 * The function returns the portion of the path preceding the last component
 * (e.g. for "/a/b/c" it returns "/a/b"). If no slash is found, an empty
 * string is returned.
 */
std::string getInterfaceUpstreamConfig(const std::string& fullInterfaceName);

/**
 * @brief Parse the integer index suffix from a prefix like "joint_3".
 *
 * Returns the numeric suffix after the last underscore.
 */
inline int getJointIndexFromPrefix(const std::string& prefix) {
   std::string jointIndexStr = prefix.substr(prefix.find_last_of("_") + 1);
   return std::stoi(jointIndexStr);
}

/**
 * @brief Fill a JointTrajectoryPoint with provided state vectors and timestamp.
 *
 * Helper used to populate a trajectory point's positions, velocities,
 * accelerations and efforts from the given vectors.
 */
bool assignState(const rclcpp::Time& stampTime,
                 trajectory_msgs::msg::JointTrajectoryPoint& trajPoint,
                 const std::vector<double>& positions,
                 const std::vector<double>& velocities,
                 const std::vector<double>& accelerations,
                 const std::vector<double>& torques);

/**
 * @brief Retry a function until success or maximum attempts are reached.
 *
 * Optionally logs a message on each failed attempt.
 */
bool retry(const std::function<bool()>& func, size_t maxAttempts,
           std::chrono::milliseconds delayBetweenAttempts,
           std::optional<std::string> logMsg = std::nullopt);

/**
 * @brief Compute absolute error between two scalar values.
 */
double error(const double a, const double b);

/**
 * @brief Compute the maximum absolute element-wise error between two vectors.
 */
double maxError(const std::vector<double>& as, const std::vector<double>& bs);

/**
 * @brief Compute element-wise error vector between two vectors.
 */
std::vector<double> calculateError(const std::vector<double>& v1,
                                   const std::vector<double>& v2);

/**
 * @brief Compute the error between two trajectory points, returning a
 * trajectory point of errors.
 */
trajectory_msgs::msg::JointTrajectoryPoint calculateError(
       const rclcpp::Time& timeFromStart,
       const trajectory_msgs::msg::JointTrajectoryPoint& p1,
       const trajectory_msgs::msg::JointTrajectoryPoint& p2);

}  // namespace controller_utils::utils
#endif  // CONTROLLER_UTILS___UTILS_HPP_
