cmake_minimum_required(VERSION 3.8)
project(power_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  PROGRAMS
    power_monitor/battery_status.py
    power_monitor/motor_status.py
    power_monitor/fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
