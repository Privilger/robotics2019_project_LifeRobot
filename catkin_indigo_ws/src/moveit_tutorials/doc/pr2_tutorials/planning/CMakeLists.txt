find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
control_msgs
)

add_executable(planning_scene_tutorial src/planning_scene_tutorial.cpp)
target_link_libraries(planning_scene_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene_ros_api_tutorial src/planning_scene_ros_api_tutorial.cpp)
target_link_libraries(planning_scene_ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(motion_planning_api_tutorial src/motion_planning_api_tutorial.cpp)
target_link_libraries(motion_planning_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_pipeline_tutorial src/planning_pipeline_tutorial.cpp)
target_link_libraries(planning_pipeline_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_pipeline_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(fetch_test src/fetch_test.cpp)
target_link_libraries(fetch_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS fetch_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(fetch_add_collision src/fetch_add_collision.cpp)
target_link_libraries(fetch_add_collision ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS fetch_add_collision DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(fetch_func_test src/fetch_func_test.cpp)
target_link_libraries(fetch_func_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS fetch_func_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
