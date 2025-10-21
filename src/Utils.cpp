#include <cmath>
#include "controller_utils/Utils.hpp"

namespace controller_utils::utils {

std::string getInterfaceTypeFromFullName(const std::string& fullInterfaceName) {
   size_t lastSlashPos = fullInterfaceName.find_last_of("/");
   if (lastSlashPos == std::string::npos) {
      return fullInterfaceName;
   }
   return fullInterfaceName.substr(lastSlashPos + 1);
}

std::string getInterfaceUpstreamConfig(const std::string& fullInterfaceName) {
   size_t lastSlashPos = fullInterfaceName.find_last_of("/");
   if (lastSlashPos == std::string::npos) {
      return "";
   }

   size_t secondLastSlashPos =
          fullInterfaceName.find_last_of("/", lastSlashPos - 1);
   if (secondLastSlashPos == std::string::npos) {
      return fullInterfaceName.substr(0, lastSlashPos + 1);
   }

   return fullInterfaceName.substr(
          0, secondLastSlashPos + 1);  // +1 to include the second to last slash
}

bool assignState(const rclcpp::Time& timeFromStart,
                 trajectory_msgs::msg::JointTrajectoryPoint& trajPoint,
                 const std::vector<double>& positions,
                 const std::vector<double>& velocities,
                 const std::vector<double>& accelerations,
                 const std::vector<double>& torques) {
   auto assignOrNan = [](const std::vector<double>& values) {
      if (values.empty()) {
         return std::vector<double>(1, std::nan(""));
      }
      return values;
   };

   trajPoint.time_from_start =
          rclcpp::Duration::from_nanoseconds(timeFromStart.nanoseconds());
   trajPoint.positions = assignOrNan(positions);
   trajPoint.velocities = assignOrNan(velocities);
   trajPoint.accelerations = assignOrNan(accelerations);
   trajPoint.effort = assignOrNan(torques);
   return true;
}

std::vector<double> calculateError(const std::vector<double>& v1,
                                   const std::vector<double>& v2) {
   std::vector<double> error;
   error.reserve(v2.size());
   for (size_t i = 0; i < v2.size(); i++) {
      error.push_back(v2[i] - v1[i]);
   }
   return error;
}

trajectory_msgs::msg::JointTrajectoryPoint calculateError(
       const rclcpp::Time& timeFromStart,
       const trajectory_msgs::msg::JointTrajectoryPoint& p1,
       const trajectory_msgs::msg::JointTrajectoryPoint& p2) {
   trajectory_msgs::msg::JointTrajectoryPoint errorPoint;
   errorPoint.time_from_start =
          rclcpp::Duration::from_nanoseconds(timeFromStart.nanoseconds());
   errorPoint.positions = calculateError(p1.positions, p2.positions);
   errorPoint.velocities = calculateError(p1.velocities, p2.velocities);
   errorPoint.accelerations =
          calculateError(p1.accelerations, p2.accelerations);
   errorPoint.effort = calculateError(p1.effort, p2.effort);
   return errorPoint;
}

double error(const double a, const double b) {
   return a - b;
}

double maxError(const std::vector<double>& as, const std::vector<double>& bs) {
   std::vector<double> errors(as.size());
   std::transform(as.begin(), as.end(), bs.begin(), errors.begin(), error);
   return *std::max_element(errors.begin(), errors.end());
}

bool retry(const std::function<bool()>& func, size_t maxAttempts,
           std::chrono::milliseconds delayBetweenAttempts,
           std::optional<std::string> logMsg) {
   for (size_t attempt{0}; attempt < maxAttempts; ++attempt) {
      if (func()) {
         return true;
      }
      std::this_thread::sleep_for(delayBetweenAttempts);
      if (logMsg.has_value()) {
         spdlog::info("Retry attempt {}. {}", attempt, logMsg.value());
      }
   }
   return false;
}
}  // namespace controller_utils::utils
