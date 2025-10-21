#ifndef CONTROLLER_UTILS___KINEMATIC_INTERFACES__INTERFACECONTAINER_HPP_
#define CONTROLLER_UTILS___KINEMATIC_INTERFACES__INTERFACECONTAINER_HPP_

#include <controller_interface/helpers.hpp>
#include "controller_utils/InterfaceRegistry.hpp"

namespace controller_utils::interfaces {

/**
 * @brief Registry for declared interfaces and their clamed handles.
 *
 * This template collects interface identifiers (component + type), preserves
 * their declared ordering, and binds those identifiers to actual interface
 * handles when available. It exposes utilities to register expected interfaces,
 * resolve and claim unordered handles into an ordered view, and read or write
 * values through those handles.
 *
 * Identifiers follow the convention "componentName/interfaceType", where
 * componentName is an arbitrary label (for example "controller_1/joint_1") and
 * interfaceType classifies the interface (for example "position" or
 * "velocity").
 *
 * @tparam T Underlying interface handle type.
 */
template <typename T>
class RegistryInterface {
   public:
   /**
    * @brief Declare (register) an expected interface entry using a component
    * prefix and type.
    *
    * If the requested interface type exists in the provided configuration, an
    * ordered identifier is added to the registry. Missing non-optional types
    * cause an error and return false; missing optional types log a debug
    * message and registration continues.
    *
    * @param componentPrefix Prefix to prepend to the component name when
    * forming identifiers.
    * @param interfaceConfig Vector of configured interface types to check
    * against.
    * @param interfaceType Interface type to register (e.g. "position").
    * @param errorMessage Message logged on fatal (non-optional) absence.
    * @param warningMessage Message logged when an optional interface is absent.
    * @param isInterfaceOptional Whether the interface may be missing without
    * failing.
    * @return true on success, false if a required interface was not found.
    */
   bool registerInterface(const std::string& componentPrefix,
                          const std::vector<std::string>& interfaceConfig,
                          const std::string& interfaceType,
                          const char* errorMessage, const char* warningMessage,
                          bool isInterfaceOptional) {
      if (containsInterfaceType(interfaceConfig, interfaceType)) {
         mOrderedRegistryNames.emplace_back(componentPrefix, interfaceType);
      } else {
         if (!isInterfaceOptional) {
            spdlog::error(errorMessage);
            return false;
         } else {
            spdlog::debug(warningMessage);
         }
      }
      return true;
   }

   /**
    * @brief Return the ordered list of full interface identifiers (e.g.
    * "joint_1/position").
    *
    * @return A vector containing the full names for each registered interface.
    */
   std::vector<std::string> getRegistryFullNames() const {
      std::vector<std::string> fullNames;
      fullNames.reserve(mOrderedRegistryNames.size());
      for (const auto& entry : mOrderedRegistryNames) {
         fullNames.push_back(entry.fullName);
      }
      return fullNames;
   }

   /**
    * @brief Return the interface types for every registered entry (e.g.
    * "position").
    *
    * @return A vector of interface type strings in the same order as
    * registration.
    */
   std::vector<std::string> getRegistryTypes() const {
      std::vector<std::string> names;
      names.reserve(mOrderedRegistryNames.size());
      for (const auto& entry : mOrderedRegistryNames) {
         names.push_back(entry.interfaceType);
      }
      return names;
   }

   /**
    * @brief Return the component portion of each registered identifier (e.g.
    * "joint_1").
    *
    * @return A vector of component name strings, ordered to match the registry.
    */
   std::vector<std::string> getInterfaceNames() const {
      std::vector<std::string> names;
      names.reserve(mOrderedRegistryNames.size());
      for (const auto& entry : mOrderedRegistryNames) {
         names.push_back(entry.componentName);
      }
      return names;
   }

   /**
    * @brief Match and bind unordered handles into the registry's ordered view.
    *
    * Uses controller_interface helper to match available handles to the
    * registry's full names and stores ordered references to the matched
    * handles.
    *
    * @param unorderedInterfaces A collection of available, unordered interface
    * handles.
    * @return true when Claiming succeeds for all registered names, false
    * otherwise.
    */
   bool claimInterfaces(std::vector<T>& unorderedInterfaces) {
      if (!controller_interface::get_ordered_interfaces(
                 unorderedInterfaces, getRegistryFullNames(), "",
                 mOrderedRegistries)) {
         return false;
      }
      return true;
   }

   /**
    * @brief Return how many interfaces have been registered.
    *
    * This count represents the number of degrees of freedom for this
    * specific interface category.
    *
    * @return Number of registered interfaces.
    */
   size_t getRegistrySize() const {
      return mOrderedRegistryNames.size();
   }

   /**
    * @brief Read numeric values from every clamed interface handle.
    *
    * @return A vector with the current value of each interface in registry
    * order.
    */
   std::vector<double> getValues() const {
      std::vector<double> values(mOrderedRegistries.size());
      for (size_t i = 0; i < mOrderedRegistries.size(); i++) {
         values[i] = mOrderedRegistries[i].get().get_value();
      }

      return values;
   }

   /**
    * @brief Write numeric values to every clamed interface handle.
    *
    * The provided vector must match the registry size.
    *
    * @param values Values to write, ordered to match the registry.
    * @return true on success, false when provided size mismatches.
    */
   bool setValues(const std::vector<double>& values) {
      if (mOrderedRegistries.size() != values.size()) {
         return false;
      }

      auto valueIt = values.begin();
      for (auto& interface : mOrderedRegistries) {
         interface.get().set_value(*valueIt++);
      }
      return true;
   }

   /**
    * @brief Remove all clamed interface handles from the registry.
    */
   void unclaimRegistry() {
      mOrderedRegistries.clear();
   }

   /**
    * @brief Remove all registered interface name entries.
    */
   void clearRegistryNames() {
      mOrderedRegistryNames.clear();
   }

   /**
    * @brief Unregister both names and clamed handles.
    */
   void unregisterInterfaces() {
      unclaimRegistry();
      clearRegistryNames();
   }

   /**
    * @brief Access the clamed interface handle at the given index.
    *
    * @param index Zero-based index into the ordered registry.
    * @return Reference to the underlying interface handle.
    */
   T& operator[](size_t index) {
      return mOrderedRegistries[index].get().get_value();
   }

   private:
   std::vector<controller_utils::interfaces::InterfaceRegistry>
          mOrderedRegistryNames;
   std::vector<std::reference_wrapper<T>> mOrderedRegistries;
};

}  // namespace controller_utils::interfaces

#endif  // CONTROLLER_UTILS___KINEMATIC_INTERFACES__INTERFACECONTAINER_HPP_
