#ifndef CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICSTATEINTERFACES_HPP_
#define CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICSTATEINTERFACES_HPP_

#include "controller_utils/kinematic_interfaces/JointInterfaces.hpp"

namespace controller_utils::interfaces {

struct OptionalStateInterfaces : public OptionalInterfaces {
   bool errorCodes =
          false; /**< When true, error code interfaces are optional. */
   bool accelerations =
          false; /**< When true, acceleration interfaces are optional. */
};

/**
 * @class KinematicStateInterfaces
 * @brief State-oriented extensions for joint state collections.
 *
 * Builds on `JointInterfaces<LoanedStateInterface>` by adding optional state
 * channels such as accelerations and error codes. Provides methods to register
 * expected state channels, bind available handles into the ordered registry,
 * query the composed configuration, and read acceleration and error-code
 * values.
 */
class KinematicStateInterfaces
         : public JointInterfaces<hardware_interface::LoanedStateInterface> {
   public:
   /**
    * @brief Register state interfaces for a component prefix.
    *
    * Registers acceleration and error-code channels in addition to the base
    * joint interfaces. Optional flags determine whether missing channels are
    * tolerated.
    */
   bool registerStateInterfaces(const std::string& componentPrefix,
                                const std::vector<std::string>& interfaceConfig,
                                const OptionalStateInterfaces& optionalFlags);

   /**
    * @brief Return the combined configuration of state interface identifiers.
    *
    * This includes the base joint interface identifiers (positions, velocities,
    * torques) followed by any registered acceleration and error-code
    * identifiers.
    */
   std::vector<std::string> stateInterfaceConfiguration() const;

   /**
    * @brief Match and bind unordered state handles into the registry order.
    *
    * Returns true when all required entries were found and clamed successfully.
    */
   bool claimInterfaces(std::vector<hardware_interface::LoanedStateInterface>&
                              unorderedInterfaces);

   /**
    * @brief Read acceleration values from clamed acceleration interfaces.
    *
    * @return Vector of acceleration values in registry order.
    */
   std::vector<double> getAccelerationValues() const;

   /**
    * @brief Read error-code values from clamed error-code interfaces.
    *
    * @return Vector of error-code values in registry order.
    */
   std::vector<double> getErrorCodeValues() const;

   /**
    * @brief Remove clamed state handles but keep registered names.
    */
   void unclaimStateInterfaces();

   /**
    * @brief Unregister names and remove any clamed state handles.
    */
   void unregisterStateInterfaces();

   /**
    * @brief Reset the registered state interface names (prepare for
    * reconfiguration).
    */
   void resetStateInterfaces();

   private:
   RegistryInterface<hardware_interface::LoanedStateInterface> mErrorCodes;
   RegistryInterface<hardware_interface::LoanedStateInterface> mAccelerations;
};

}  // namespace controller_utils::interfaces

#endif  // CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICSTATEINTERFACES_HPP_
