cmake_minimum_required(VERSION 3.5)
project(opti_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(NatNet_DIR C:/NatNet_SDK_4.0/NatNetSDK/)

add_library(natnet_sdk SHARED IMPORTED)
target_include_directories(natnet_sdk INTERFACE ${NatNet_DIR}/include)
set_target_properties(natnet_sdk 
PROPERTIES 
  IMPORTED_IMPLIB ${NatNet_DIR}/lib/x64/NatNetLib.lib
  IMPORTED_LOCATION ${NatNet_DIR}/lib/x64/NatNetLib.dll
)

include_directories(
  include/opti_driver
)

add_executable(Opti_Node src/OptiTrack_Driver.cpp)

target_include_directories(Opti_Node
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(Opti_Node
PUBLIC
  natnet_sdk
  rclcpp::rclcpp
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
)

install(TARGETS
  Opti_Node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES                 ${NatNet_DIR}/lib/x64/NatNetLib.lib
  DESTINATION           lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
