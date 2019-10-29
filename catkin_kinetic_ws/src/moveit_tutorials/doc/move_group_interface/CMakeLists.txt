add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(move_group_test src/move_group_test.cpp)
target_link_libraries(move_group_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
