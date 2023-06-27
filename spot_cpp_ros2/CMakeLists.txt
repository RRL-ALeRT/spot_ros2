cmake_minimum_required(VERSION 3.14)
project(spot_cpp_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.54.2
)
set(FETCHCONTENT_QUIET ON)
FetchContent_MakeAvailable(gRPC)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(CLI11 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(get_image src/get_image.cpp)
target_link_directories(get_image PUBLIC lib)
ament_target_dependencies(get_image CLI11 OpenCV rclcpp sensor_msgs tf2_ros PCL pcl_msgs pcl_conversions cv_bridge image_transport)
target_link_libraries(get_image grpc++ bosdyn_api bosdyn_api_static bosdyn_client bosdyn_client_static bosdyn_choreography_protos bosdyn_choreography_protos_static pcl_common pcl_filters pcl_io)
target_include_directories(get_image PUBLIC include/bosdyn/protos)

install(TARGETS get_image
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
