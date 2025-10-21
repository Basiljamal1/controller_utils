
#include "controller_utils/kinematic_interfaces/KinematicStateInterfaces.hpp"

namespace controller_utils::interfaces {

bool KinematicStateInterfaces::registerStateInterfaces(
       const std::string& componentPrefix,
       const std::vector<std::string>& interfaceConfig,
       const OptionalStateInterfaces& optionalFlags) {
   bool success = registerBaseInterfaces(componentPrefix, interfaceConfig,
                                         optionalFlags);
   success &= mErrorCodes.registerInterface(
          componentPrefix, interfaceConfig, ERROR_CODE_INTERFACE_NAME,
          "The error code state interface is not found!",
          "The error code state interface is not found and will not be used!",
          optionalFlags.errorCodes);
   success &= mAccelerations.registerInterface(
          componentPrefix, interfaceConfig, ACCELERATION_INTERFACE_NAME,
          "The acceleration state interface is not found!",
          "The acceleration state interface is not found and will not be used!",
          optionalFlags.accelerations);

   return success;
}

std::vector<std::string> KinematicStateInterfaces::stateInterfaceConfiguration()
       const {
   auto stateInterfaceConfig = JointInterfaces::baseInterfaceConfiguration();
   for (auto& interfaceFullNames : mErrorCodes.getRegistryFullNames()) {
      stateInterfaceConfig.push_back(std::move(interfaceFullNames));
   }
   for (auto& interfaceFullNames : mAccelerations.getRegistryFullNames()) {
      stateInterfaceConfig.push_back(std::move(interfaceFullNames));
   }
   return stateInterfaceConfig;
}

bool KinematicStateInterfaces::claimInterfaces(
       std::vector<hardware_interface::LoanedStateInterface>&
              unorderedInterfaces) {
   if (!JointInterfaces::claimInterfaces(unorderedInterfaces)) {
      return false;
   }

   if (!mErrorCodes.claimInterfaces(unorderedInterfaces)) {
      printWantedVsAvailableInterfaces(unorderedInterfaces,
                                       mErrorCodes.getRegistryFullNames());
      return false;
   }

   if (!mAccelerations.claimInterfaces(unorderedInterfaces)) {
      printWantedVsAvailableInterfaces(unorderedInterfaces,
                                       mAccelerations.getRegistryFullNames());
      return false;
   }

   return true;
}

void KinematicStateInterfaces::unclaimStateInterfaces() {
   JointInterfaces::unclaimBaseInterfaces();
   mErrorCodes.unclaimRegistry();
   mAccelerations.unclaimRegistry();
}

void KinematicStateInterfaces::unregisterStateInterfaces() {
   JointInterfaces::unregisterBaseInterfaces();
   mErrorCodes.unregisterInterfaces();
   mAccelerations.unregisterInterfaces();
}

void KinematicStateInterfaces::resetStateInterfaces() {
   JointInterfaces::resetBaseInterfaces();
   mErrorCodes.clearRegistryNames();
   mAccelerations.clearRegistryNames();
}

std::vector<double> KinematicStateInterfaces::getAccelerationValues() const {
   return mAccelerations.getValues();
}

std::vector<double> KinematicStateInterfaces::getErrorCodeValues() const {
   return mErrorCodes.getValues();
}

};  // namespace controller_utils::interfaces
