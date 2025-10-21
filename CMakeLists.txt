cmake_minimum_required(VERSION 3.5)
project(controller_utils)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PACKAGE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    rcpputils
    controller_interface
    hardware_interface
)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
    src/Utils.cpp
    src/InterfaceRegistry.cpp
    src/kinematic_interfaces/KinematicStateInterfaces.cpp
    src/kinematic_interfaces/JointReferenceInterface.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Sets the visibility macro to use dllexport when building the library,
# ensuring correct symbol export for shared library creation.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    "${PROJECT_NAME_UPPER}_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rcpputils
  controller_interface
  hardware_interface
)

ament_package()
