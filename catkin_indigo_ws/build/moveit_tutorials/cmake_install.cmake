# Install script for directory: /home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/src/moveit_tutorials

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/catkin_generated/installspace/moveit_tutorials.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_tutorials/cmake" TYPE FILE FILES
    "/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/catkin_generated/installspace/moveit_tutorialsConfig.cmake"
    "/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/catkin_generated/installspace/moveit_tutorialsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/moveit_tutorials" TYPE FILE FILES "/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/src/moveit_tutorials/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/doc/pr2_tutorials/kinematics/cmake_install.cmake")
  include("/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/doc/pr2_tutorials/planning/cmake_install.cmake")
  include("/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/doc/pr2_tutorials/state_display/cmake_install.cmake")
  include("/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/doc/pr2_tutorials/interactivity/cmake_install.cmake")
  include("/home/fetch/robotics2019_project_liferrobot/catkin_indigo_ws/build/moveit_tutorials/doc/pr2_tutorials/pick_place/cmake_install.cmake")

endif()

