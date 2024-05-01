cmake_minimum_required(VERSION 3.8)
project(kindhelm_rtk_ros_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include ${BOOST_INCLUDE_DIRS})


add_executable(tiny src/tiny.cpp)
target_link_libraries(tiny ${catkin_LIBRARIES})
ament_target_dependencies(tiny rclcpp sensor_msgs)

install(TARGETS
        tiny
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)


ament_package()
