#ifndef CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICINTERFACES_HPP_
#define CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICINTERFACES_HPP_

#include <spdlog/spdlog.h>
#include <string>
#include <vector>
#include <sstream>

#include <controller_interface/helpers.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include "controller_utils/kinematic_interfaces/RegistryInterface.hpp"

namespace controller_utils::interfaces {
template <typename InterfaceType>
class JointInterfaces;

/**
 * @brief Log available interface names alongside requested ones for
 * diagnostics.
 *
 * Dumps the names provided by the controller manager and the list of
 * identifiers that the client or controller requested. Useful for debugging
 * interface mismatches during configuration or claiming.
 *
 * @tparam InterfaceType Type of the interfaces in the provided collection.
 * @param availableInterfaces Collection of interfaces currently provided by the
 * manager.
 * @param wantedInterfaces Identifiers requested by the client/controller.
 */
template <typename InterfaceType>
void printWantedVsAvailableInterfaces(
       const InterfaceType& availableInterfaces,
       const std::vector<std::string>& wantedInterfaces) {
   std::stringstream ss;
   for (const auto& interface : availableInterfaces) {
      ss << interface.get_name() << " ";
   }

   std::stringstream ss2;
   for (const auto& interface : wantedInterfaces) {
      ss2 << interface << " ";
   }
   spdlog::error("Available interfaces: {}", ss.str());
   spdlog::error("Wanted interfaces: {}", ss2.str());
}

/**
 * @brief Flags indicating which kinematic interface types are optional.
 *
 * Each boolean toggles whether its corresponding interface may be absent
 * without causing registration to fail.
 */
struct OptionalInterfaces {
   bool position = false; /**< true when position interfaces are optional. */
   bool velocity = false; /**< true when velocity interfaces are optional. */
   bool torque = false;   /**< true when torque interfaces are optional. */
};

template <typename InterfaceType>
class JointInterfaces {
   public:
   /**
    * @brief Register the expected base interfaces using a component prefix.
    *
    * Builds registry entries such as "joint_1/position" for each configured
    * interface type. Required interfaces must exist in the provided
    * configuration; optional ones are tolerated and logged.
    *
    * @param componentPrefix Prefix for component identifiers (e.g. "joint_1").
    * @param interfaceConfig List of available interface type names (e.g.
    * "position").
    * @param optionalInterfaces Flags marking which interface types are
    * optional.
    * @return true when registration succeeds for all required interfaces.
    */
   bool registerBaseInterfaces(const std::string& componentPrefix,
                               const std::vector<std::string>& interfaceConfig,
                               const OptionalInterfaces& optionalInterfaces) {
      bool success = true;
      success &= mPositions.registerInterface(
             componentPrefix, interfaceConfig, POSITION_INTERFACE_NAME,
             "The position interface is not found!",
             "The position interface is not found and will not be used!",
             optionalInterfaces.position);

      success &= mVelocities.registerInterface(
             componentPrefix, interfaceConfig, VELOCITY_INTERFACE_NAME,
             "The velocity interface is not found!",
             "The velocity interface is not found and will not be used!",
             optionalInterfaces.velocity);

      success &= mTorques.registerInterface(
             componentPrefix, interfaceConfig, TORQUE_INTERFACE_NAME,
             "The torque interface is not found!",
             "The torque interface is not found and will not be used!",
             optionalInterfaces.torque);

      return success;
   }
   /**
    * @brief Return the list of base interface types that were registered.
    *
    * The returned vector contains the interface type names in the same order
    * they were added to the registry.
    *
    * @return Vector of registered base interface type names.
    */
   std::vector<std::string> baseInterfaceConfiguration() const {
      const size_t stateSize = mPositions.getRegistrySize() +
                               mVelocities.getRegistrySize() +
                               mTorques.getRegistrySize();
      std::vector<std::string> stateInterfaceConfig;
      stateInterfaceConfig.reserve(stateSize);
      for (auto& interfaceFullName : mPositions.getRegistryFullNames()) {
         stateInterfaceConfig.push_back(std::move(interfaceFullName));
      }
      for (auto& interfaceFullName : mVelocities.getRegistryFullNames()) {
         stateInterfaceConfig.push_back(std::move(interfaceFullName));
      }
      for (auto& interfaceFullName : mTorques.getRegistryFullNames()) {
         stateInterfaceConfig.push_back(std::move(interfaceFullName));
      }
      return stateInterfaceConfig;
   }
   /**
    * @brief Resolve available handles and bind them into the registry's order.
    *
    * Matches the provided unordered handles to the registry's full names and
    * stores ordered references so values can be read or written in a stable
    * order.
    *
    * @param unorderedInterfaces Collection of available, unordered interface
    * handles.
    * @return true when matching and binding succeeds for all required entries.
    */
   bool claimInterfaces(std::vector<InterfaceType>& unorderedInterfaces) {
      if (!mPositions.claimInterfaces(unorderedInterfaces)) {
         printWantedVsAvailableInterfaces(unorderedInterfaces,
                                          mPositions.getRegistryFullNames());
         return false;
      }

      if (!mVelocities.claimInterfaces(unorderedInterfaces)) {
         printWantedVsAvailableInterfaces(unorderedInterfaces,
                                          mVelocities.getRegistryFullNames());
         return false;
      }

      if (!mTorques.claimInterfaces(unorderedInterfaces)) {
         printWantedVsAvailableInterfaces(unorderedInterfaces,
                                          mTorques.getRegistryFullNames());
         return false;
      }

      return true;
   }
   /**
    * @brief Remove clamed handles but keep the registered names.
    *
    * Use this to temporarily release claimed interfaces while preserving the
    * configuration so they can be re-clamed later.
    */
   void unclaimBaseInterfaces() {
      mPositions.unclaimRegistry();
      mVelocities.unclaimRegistry();
      mTorques.unclaimRegistry();
   }
   /**
    * @brief Reset internal state related to base interfaces.
    *
    * Prepares the object to accept a different interface configuration.
    */
   void resetBaseInterfaces() {
      mPositions.clearRegistryNames();
      mVelocities.clearRegistryNames();
      mTorques.clearRegistryNames();
   }

   /**
    * @brief Remove registered interface names and any clamed handles.
    *
    * Use when the controller intends to change the configured interfaces.
    */
   void unregisterBaseInterfaces() {
      mPositions.unregisterInterfaces();
      mVelocities.unregisterInterfaces();
      mTorques.unregisterInterfaces();
   }
   /**
    * @brief Read current position values from all clamed position interfaces.
    *
    * @return Vector of position values in registry order.
    */
   std::vector<double> getPositionValues() const {
      return mPositions.getValues();
   }
   /**
    * @brief Read current velocity values from all clamed velocity interfaces.
    *
    * @return Vector of velocity values in registry order.
    */
   std::vector<double> getVelocityValues() const {
      return mVelocities.getValues();
   }
   /**
    * @brief Read current torque (effort) values from all clamed torque
    * interfaces.
    *
    * @return Vector of torque values in registry order.
    */
   std::vector<double> getTorqueValues() const {
      return mTorques.getValues();
   }
   /**
    * @brief Write a set of position values to the clamed position interfaces.
    *
    * @param values Position values to write, in registry order.
    * @return true when all values were applied successfully.
    */
   bool setPositionValues(const std::vector<double>& values) {
      return mPositions.setValues(values);
   }
   /**
    * @brief Write a set of velocity values to the clamed velocity interfaces.
    *
    * @param values Velocity values to write, in registry order.
    * @return true when all values were applied successfully.
    */
   bool setVelocityValues(const std::vector<double>& values) {
      return mVelocities.setValues(values);
   }
   /**
    * @brief Write a set of torque values to the clamed torque interfaces.
    *
    * @param values Torque (effort) values to write, in registry order.
    * @return true when all values were applied successfully.
    */
   bool setTorqueValues(const std::vector<double>& values) {
      return mTorques.setValues(values);
   }
   /**
    * @brief Retrieve component names for each registered position interface.
    *
    * @return A vector with component name strings in registry order.
    */
   std::vector<std::string> getPositionComponentNames() const {
      return mPositions.getInterfaceNames();
   }
   /**
    * @brief Retrieve component names for each registered velocity interface.
    *
    * @return A vector with component name strings in registry order.
    */
   std::vector<std::string> getVelocityComponentNames() const {
      return mVelocities.getInterfaceNames();
   }
   /**
    * @brief Retrieve component names for each registered torque interface.
    *
    * @return A vector with component name strings in registry order.
    */
   std::vector<std::string> getTorqueComponentNames() const {
      return mTorques.getInterfaceNames();
   }

   private:
   RegistryInterface<InterfaceType>
          mPositions; /**< Holds registered position interfaces. */
   RegistryInterface<InterfaceType>
          mVelocities; /**< Holds registered velocity interfaces. */
   RegistryInterface<InterfaceType>
          mTorques; /**< Holds registered torque interfaces. */
};

}  // namespace controller_utils::interfaces

#endif  // CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICINTERFACES_HPP_
