cmake_minimum_required(VERSION 3.5)
project(lego_loam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  #set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()
# SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
set(BOOST_DIR "/usr/lib/x86_64-linux-gnu/cmake/Boost-1.71.0/BoostConfig.cmake")

find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED iostreams date_time serialization thread program_options regex)
ament_auto_find_build_dependencies()
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
	include
	${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${EIGEN3_LIBRARY_DIRS}
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(image_projection src/image_projection.cpp)
ament_target_dependencies(image_projection
rclcpp 
rclcpp_components
cloud_msgs
tf2_ros 
tf2_geometry_msgs 
tf2_sensor_msgs 
tf2_eigen 
geometry_msgs 
sensor_msgs
nav_msgs
PCL
OpenCV)
# rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(image_projection  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(feature_association src/feature_association.cpp)
# add_dependencies(featureAssociation  cloud_msgs rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs)
ament_target_dependencies(feature_association
rclcpp 
rclcpp_components
cloud_msgs
tf2_ros 
tf2_geometry_msgs 
tf2_sensor_msgs 
tf2_eigen 
geometry_msgs 
sensor_msgs
nav_msgs
PCL
OpenCV)
# rosidl_target_interfaces(${PROJECT_NAME}_featureAssociation ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(feature_association ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} glog)

# add_executable(mapOptmization src/mapOptmization.cpp)
# add_dependencies(featureAssociation  cloud_msgs)
# rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(mapOptmization ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# add_dependencies(featureAssociation  cloud_msgs)
# add_executable(transformFusion src/transformFusion.cpp)
# rosidl_target_interfaces(${PROJECT_NAME}_transformFusion ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(transformFusion ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS image_projection
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   TARGETS ${PROJECT_NAME}_transformFusion
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  TARGETS feature_association
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   TARGETS ${PROJECT_NAME}_mapOptimization
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

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

