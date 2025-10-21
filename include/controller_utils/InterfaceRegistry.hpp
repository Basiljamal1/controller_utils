#ifndef CONTROLLER_UTILS___INTERFACES_HPP_
#define CONTROLLER_UTILS___INTERFACES_HPP_

#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace controller_utils::interfaces {
   
constexpr char FEEDFORWARD_VELOCITY_INTERFACE_NAME[] = "feedforward_velocity";
constexpr char TORQUE_INTERFACE_NAME[] = "torque";
constexpr char ERROR_CODE_INTERFACE_NAME[] = "error_code";
constexpr char CLEAR_FAULT_INTERFACE_NAME[] = "clear_fault";
constexpr char POSITION_INTERFACE_NAME[] = "position";
constexpr char VELOCITY_INTERFACE_NAME[] = "velocity";
constexpr char ACCELERATION_INTERFACE_NAME[] = "acceleration";

constexpr auto STOP_VELOCITY = 0;
constexpr auto STOP_ACCELERATION = 0;
constexpr auto STOP_TORQUE = 0;

/**
 * @struct InterfaceRegistry
 * @brief Simple record representing a declared interface identifier.
 *
 * Holds the component portion, the interface type, and the composed
 * full name "component/type".
 */
struct InterfaceRegistry {
   InterfaceRegistry(const std::string& componentName,
                     const std::string& interfaceType)
            : componentName(componentName),
              interfaceType(interfaceType),
              fullName(componentName + "/" + interfaceType) {
   }

   InterfaceRegistry(InterfaceRegistry&& other) noexcept
            : componentName(std::move(other.componentName)),
              interfaceType(std::move(other.interfaceType)),
              fullName(std::move(other.fullName)) {
   }

   const std::string componentName; /**< component label (e.g. "joint_1") */
   const std::string interfaceType; /**< interface type (e.g. "position") */
   const std::string fullName;      /**< composed string "component/type" */
};

inline bool operator==(const std::vector<InterfaceRegistry>& lhs,
                       const std::vector<InterfaceRegistry>& rhs) {
   if (lhs.size() != rhs.size()) {
      return false;
   }

   for (size_t i = 0; i < lhs.size(); ++i) {
      if (lhs[i].componentName != rhs[i].componentName ||
          lhs[i].interfaceType != rhs[i].interfaceType ||
          lhs[i].fullName != rhs[i].fullName) {
         return false;
      }
   }

   return true;
}

/**
 * @struct InterfaceRegistryLink
 * @brief Holds a declared interface identifier and a pointer to a matching
 * handle.
 *
 * The template binds an `InterfaceRegistry` identifier to a raw pointer of the
 * corresponding handle type when a match is found.
 */
template <typename T>
struct InterfaceRegistryLink {
   InterfaceRegistry name;
   T* linkInterface = nullptr;

   InterfaceRegistryLink();
   InterfaceRegistryLink(const std::string componentName,
                         const std::string interfaceType)
            : name(componentName, interfaceType) {
   }

   InterfaceRegistryLink(InterfaceRegistryLink&& other) noexcept
            : name(std::move(other.name)), linkInterface(other.linkInterface) {
      other.linkInterface = nullptr;
   }

   InterfaceRegistryLink& operator=(InterfaceRegistryLink&& other) noexcept {
      if (this != &other) {
         linkInterface = other.linkInterface;
         other.linkInterface = nullptr;
      }
      return *this;
   }

   InterfaceRegistryLink(const InterfaceRegistryLink&) = delete;
   InterfaceRegistryLink& operator=(const InterfaceRegistryLink&) = delete;

   std::string getFullOrderedInterfaceRegistryNames() const {
      return name.fullName;
   }

   bool linkInterfaces(std::vector<T>& unorderedInterfaces) {
      for (auto& unorderedInterface : unorderedInterfaces) {
         if (name.fullName == unorderedInterface.get_name()) {
            linkInterface = &unorderedInterface;
            return true;
         }
      }
      return false;
   }

   void unlinkInterfaces() {
      linkInterface = nullptr;
   }
};

/**
 * @brief Determine whether a candidate interface type appears in a configured
 * list.
 *
 * @param interface_type_list List of configured interface type names.
 * @param interface_type Candidate interface type to check for.
 * @return true when found, false otherwise.
 */
bool containsInterfaceType(const std::vector<std::string>& interface_type_list,
                           const std::string& interface_type);

/**
 * @brief Read numeric values from a collection of state interfaces.
 *
 * @param interfaces References to loaned state interfaces to sample.
 * @return Vector with the sampled state values in the same order as the input.
 */
std::vector<double> getStateValues(
       const std::vector<
              std::reference_wrapper<hardware_interface::LoanedStateInterface>>&
              interfaces);

/**
 * @brief Read numeric values from a collection of command interfaces.
 *
 * @param interfaces References to loaned command interfaces to sample.
 * @return Vector with the sampled command values in the same order as the
 * input.
 */
std::vector<double> getCommandValues(
       const std::vector<std::reference_wrapper<
              hardware_interface::LoanedCommandInterface>>& interfaces);

/**
 * @brief Write numeric values to a collection of command interfaces.
 *
 * The provided vector must match the number of interfaces.
 *
 * @param interfaces References to loaned command interfaces to modify.
 * @param values Values to assign to each interface, ordered to match the input.
 * @return true when the write succeeded, false when sizes mismatch.
 */
bool setCommandValues(
       std::vector<std::reference_wrapper<
              hardware_interface::LoanedCommandInterface>>& interfaces,
       const std::vector<double>& values);

}  // namespace controller_utils::interfaces

#endif  // CONTROLLER_UTILS___INTERFACES_HPP_
