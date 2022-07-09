cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file (
  ${PROJECT_SOURCE_DIR}/include/localization/global_definition/global_definition.h.in
  ${PROJECT_BINARY_DIR}/include/localization/global_definition/global_definition.h
  @ONLY
)
configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/config.h.cmake
  ${PROJECT_BINARY_DIR}/include/localization/common/config.h)

include_directories(include/${PROJECT_NAME})

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

#Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#PCL
find_package(PCL REQUIRED)
#PATHS /usr/local/pcl-1.12.1
#NO_DEFAULT_PATH)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})

#Glog
find_package(Glog REQUIRED)

#protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

#lua
find_package(Lua 5.2 REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

#GeographicLib
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include)

set(ALL_TARGET_LIBRARIES 
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  ${LUA_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  glog::glog
  libGeographiccc
)

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
  message("pb.h = " ${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h)
endforeach()

include_directories(${PROJECT_BINARY_DIR}/include/${PROJECT_NAME})
include_directories(${PROJECT_BINARY_DIR})

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

#message("ALL_SRCS = " ${ALL_SRCS})

add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS} ${ALL_PROTO_SRCS})
ament_target_dependencies(test_frame_node 
  std_msgs 
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nav_msgs
)
target_link_libraries(test_frame_node ${ALL_TARGET_LIBRARIES})

install(TARGETS
  test_frame_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
