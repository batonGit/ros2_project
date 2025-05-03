cmake_minimum_required(VERSION 3.8)
project(kinect_freenect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Найти зависимости ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Добавить исполняемый файл
add_executable(kinect_node src/kinect_node.cpp)

# Указать пути к заголовочным файлам и библиотекам
target_include_directories(kinect_node PRIVATE /usr/include/libfreenect)
target_compile_features(kinect_node PRIVATE cxx_std_17)  # Использование C++17 для поддержки лямбда-функций с захватом

# Указываем зависимости ROS
ament_target_dependencies(kinect_node
  rclcpp
  sensor_msgs
)

# Добавляем системную библиотеку libfreenect
target_link_libraries(kinect_node freenect)

# Установить исполняемый файл
install(TARGETS kinect_node
  DESTINATION lib/${PROJECT_NAME})

# Тестирование (если требуется)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()