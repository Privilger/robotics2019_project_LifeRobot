Team Name: LifeRobot <br>

Members: <br>
Yizheng Zhang: zhangyzh1@shanghaitech.edu.cn   ID: 2018233135 <br>
Jiahui  Zhu: zhujh1@shanghaitech.edu.cn      ID: 2018233141 <br>

In the field of robotics, service robots are increasingly becoming an industry hot-spot. Especially for mobilemanipulation, many researchers have studied its applications and some teams are focused on the manipulator’s poseestimation of grasping objects, and the other teams are working on the simultaneous localization and mapping, pathplanning and automatic obstacle avoidance of mobile vehicles. Very few of them explore the integrated system ofmobile manipulation and only transport simple, regular objects. Since in life science experiments, many repetitive, simple tasks can be replaced by autonomous robots. This work is aiming to apply a mobile manipulation Fetchrobot to execute the dexterous manipulation of centrifuge tubes in life science laboratories. Usually, those tubes have a screw-lid so that we need to unscrew and re-screw the lid. In this work: (a) to get an accurate centrifugetube localization; (b) to grasp the tube from the side and place it in the designated position; (c) to unscrew the lidfrom the top of the centrifuge tube and place it on the test table; (d) to re-position the lid, grasp and re-screw it. Additionally, we will use AprilTag2 and DenseFusion two pose estimation methods to grasp the centrifuge tube and the lid.


How to Reproduce the Project:

cd catkin_indigo_ws

catkin_make

roslaunch fetch_moveit_config move_group.launch

rosrun rosserial_python serial_node.py /dev/ttyUSB0

roslaunch apriltags2_ros continuous_detection.launch

roslaunch life_robot_moveit start.launch move_real_robot:=true