#include "hardware_interface/loaned_command_interface.hpp"
#include <controller_utils/InterfaceRegistry.hpp>
#include <spdlog/spdlog.h>

namespace controller_utils::interfaces {

bool setCommandValues(
       std::vector<std::reference_wrapper<
              hardware_interface::LoanedCommandInterface>>& interfaces,
       const std::vector<double>& values) {
   if (interfaces.size() != values.size()) {
      return false;
   }

   auto valueIt = values.begin();
   for (auto& interface : interfaces) {
      interface.get().set_value(*valueIt++);
   }
   return true;
}

std::vector<double> getStateValues(
       const std::vector<
              std::reference_wrapper<hardware_interface::LoanedStateInterface>>&
              interfaces) {
   std::vector<double> values(interfaces.size());
   for (size_t i = 0; i < interfaces.size(); i++) {
      values[i] = interfaces[i].get().get_value();
   }
   return values;
}

std::vector<double> getCommandValues(
       const std::vector<std::reference_wrapper<
              hardware_interface::LoanedCommandInterface>>& interfaces) {
   std::vector<double> values(interfaces.size());
   for (size_t i = 0; i < interfaces.size(); i++) {
      values[i] = interfaces[i].get().get_value();
   }
   return values;
}

bool containsInterfaceType(const std::vector<std::string>& interface_type_list,
                           const std::string& interface_type) {
   return std::find(interface_type_list.begin(), interface_type_list.end(),
                    interface_type) != interface_type_list.end();
}

}  // namespace controller_utils::interfaces
