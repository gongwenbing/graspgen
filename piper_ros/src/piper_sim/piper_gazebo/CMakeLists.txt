cmake_minimum_required(VERSION 3.5)
project(piper_gazebo)

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
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(TBB REQUIRED)
add_executable(bottle_contact_monitor src/bottle_contact_monitor.cpp)
target_include_directories(bottle_contact_monitor PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(bottle_contact_monitor ${GAZEBO_LIBRARIES} TBB::tbb)
ament_target_dependencies(bottle_contact_monitor rclcpp std_msgs)
install(TARGETS bottle_contact_monitor DESTINATION lib/${PROJECT_NAME})
add_library(piper_grasp_stabilizer SHARED src/grasp_stabilizer.cpp)
target_include_directories(piper_grasp_stabilizer PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(piper_grasp_stabilizer ${GAZEBO_LIBRARIES})
ament_target_dependencies(piper_grasp_stabilizer gazebo_ros rclcpp std_srvs)
install(TARGETS piper_grasp_stabilizer LIBRARY DESTINATION lib)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY launch config scripts worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/joint8_ctrl.py scripts/sync_pick_scene.py scripts/pick_cube.py scripts/stack_cubes.py scripts/stack_scene.py scripts/plan_vision_pregrasp.py scripts/observe_bottle.py scripts/verify_bottle_grasp.py scripts/vision_cube_to_basket.py scripts/auto_cube_task.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/bottle_model.py DESTINATION lib/${PROJECT_NAME})
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
