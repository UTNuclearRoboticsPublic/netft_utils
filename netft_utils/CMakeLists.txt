cmake_minimum_required(VERSION 3.5)
project(netft_node)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(netft_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread program_options)


include_directories(src include /usr/include/ ${Boost_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(${PROJECT_NAME} src/netft_node.cpp)
add_library(netft_rdt_driver src/netft_rdt_driver.cpp)

target_link_libraries(netft_node netft_rdt_driver)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_msgs common_interfaces geometry_msgs tf2 tf2_ros Boost)
ament_target_dependencies(netft_rdt_driver rclcpp std_msgs std_msgs common_interfaces geometry_msgs tf2 tf2_ros Boost)

install(
    TARGETS ${PROJECT_NAME} netft_rdt_driver
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()