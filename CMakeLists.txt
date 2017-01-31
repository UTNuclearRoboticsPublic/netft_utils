cmake_minimum_required(VERSION 2.8.3)
project(netft_utils)

set(GCC_COVERAGE_COMPILE_FLAGS "-g -Wno-enum-compare -std=c++11 -Wenum-compare -Wformat-security")
set(GCC_COVERAGE_LINK_FLAGS    "-Wformat-security")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

# Use C++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  diagnostic_updater 
  diagnostic_msgs
  geometry_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)

find_package(catkin REQUIRED COMPONENTS genmsg std_msgs tf message_generation)

include_directories(
  include ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder
add_message_files( FILES Cancel.msg )

add_service_files(
    FILES
    SetBias.srv
    SetFilter.srv
    SetMax.srv
    SetThreshold.srv
    SetToolData.srv
    GetDouble.srv
    StartSim.srv
    StopSim.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  LIBRARIES netft_utils_lean lpfilter
  DEPENDS
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(netft_rdt_driver src/netft_rdt_driver.cpp)
add_library(lpfilter src/lpfilter.cpp)
add_library(netft_utils_lean src/netft_utils_lean.cpp)

add_dependencies(netft_utils_lean netft_utils_generate_messages_cpp)
target_link_libraries(netft_utils_lean lpfilter netft_rdt_driver)
  
target_link_libraries(netft_rdt_driver ${Boost_LIBRARIES} ${catkin_LIBRARIES})
 
add_executable(netft_utils src/netft_utils.cpp)
add_executable(netft_utils_sim src/netft_utils_sim.cpp)
add_executable(netft_utils_cpp_test src/netft_utils_cpp_test.cpp)

add_dependencies(netft_utils netft_utils_generate_messages_cpp lpfilter)
add_dependencies(netft_utils_sim netft_utils_generate_messages_cpp lpfilter)
add_dependencies(netft_utils_cpp_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} netft_utils)

target_link_libraries(netft_utils
    ${catkin_LIBRARIES}
    lpfilter
)

target_link_libraries(netft_utils_sim
    ${catkin_LIBRARIES}
    lpfilter
)

target_link_libraries(netft_utils_cpp_test ${catkin_LIBRARIES} netft_utils_lean lpfilter netft_rdt_driver)

add_executable(netft_node src/netft_node.cpp)

target_link_libraries(netft_node netft_rdt_driver)
target_link_libraries(netft_node ${Boost_LIBRARIES})
target_link_libraries(netft_node ${catkin_LIBRARIES})

