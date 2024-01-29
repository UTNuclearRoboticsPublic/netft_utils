cmake_minimum_required(VERSION 3.5)
project(netft_interfaces)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(msg srv)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Cancel.msg
    srv/GetDouble.srv
    srv/SetBias.srv
    srv/SetFilter.srv
    srv/SetMax.srv
    srv/SetThreshold.srv
    srv/SetToolData.srv
    srv/StartSim.srv
    srv/StopSim.srv
    DEPENDENCIES builtin_interfaces geometry_msgs std_msgs common_interfaces
)

ament_package()
