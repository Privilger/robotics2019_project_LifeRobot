#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <robot_calibration_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>


#define PI 3.14159265

class FetchRobot
{
public:
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;
    actionlib::SimpleActionClient<robot_calibration_msgs::GripperLedCommandAction> led_action_client;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head_action_client;

    FetchRobot(ros::NodeHandle& nodehandle) :
            nh_(nodehandle),
            gripper_action_client("/gripper_controller/gripper_action",true),
            led_action_client("/gripper_controller/led_action",true),
            head_action_client("/head_controller/point_head",true),
            group_arm("arm"),
            group_arm_with_torso("arm_with_torso")
    {
        nh_.param("move_real_robot", moveRealRobot, false);
        ROS_INFO("Waiting for gripper action server to start.");
        gripper_action_client.waitForServer();
        ROS_INFO("Gripper action server started, sending goal.");
        ROS_INFO("Waiting for led action server to start.");
        led_action_client.waitForServer();
        ROS_INFO("Led action server started, sending goal.");
        ROS_INFO("Waiting for head action server to start.");
        head_action_client.waitForServer();
        ROS_INFO("Head action server started, sending goal.");
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

        //创建运动规划的情景，等待创建完成
        sleep(5.0);
    }

    void addFixtureToScene(double x, double y, double z)
    // x=0.23, y=0, z=0.44
    {
        moveit_msgs::CollisionObject fixture;
        fixture.header.frame_id = "base_link";
        fixture.id = "fixture";
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1; // width
        primitive.dimensions[1] = 0.15; // long
        primitive.dimensions[2] = 0.15; // height
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x =  x;
        pose.position.y =  y;
        pose.position.z =  z;
        fixture.primitives.push_back(primitive);
        fixture.primitive_poses.push_back(pose);
        fixture.operation = fixture.ADD;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(fixture);
        current_scene.addCollisionObjects(collision_objects);
    }
    void addTableToScene()
    {
        moveit_msgs::CollisionObject table, screen, wire;
        table.header.frame_id = "tag_1";
        table.id = "table";
        screen.header.frame_id = "tag_1";
        screen.id = "screen";
        wire.header.frame_id = "base_link";
        wire.id = "wire";
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.5; // width
        primitive.dimensions[1] = 1.5; // long
        primitive.dimensions[2] = 0.04; // height
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x =  -0.15;
        pose.position.y =  0.38;
        pose.position.z =  -0.02;
        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(pose);
        table.operation = table.ADD;

        primitive.dimensions[0] = 0.5; // width
        primitive.dimensions[1] = 0.5; // long
        primitive.dimensions[2] = 0.5; // height
        pose.orientation.w = 1.0;
        pose.position.x =  -0.35;
        pose.position.y =  -0.3;
        pose.position.z =  0.25;

//        primitive.dimensions[0] = 0.5; // width
//        primitive.dimensions[1] = 1.5; // long
//        primitive.dimensions[2] = 0.04; // height
//        pose.orientation.w = 1.0;
//        pose.position.x =  -0.35;
//        pose.position.y =  -0.3;
//        pose.position.z =  0.25;
        screen.primitives.push_back(primitive);
        screen.primitive_poses.push_back(pose);
        screen.operation = screen.ADD;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(table);
        collision_objects.push_back(screen);
        current_scene.addCollisionObjects(collision_objects);

    }
    void removeFixtureToScene()
    {
        std::vector<std::string> collision_objects_id;
        collision_objects_id.push_back("fixture");
        current_scene.removeCollisionObjects(collision_objects_id);
    }

    bool goToPoseGoalWithTorso(double ox, double oy, double oz, double ow, double x, double y, double z)
    // ox=0.013, oy=0.713, oz=0, ow=0.701, x=0.229, y=-0.063, z=0.672
    {
        geometry_msgs::Pose target_pose;
        target_pose.orientation.x = ox;
        target_pose.orientation.y = oy;
        target_pose.orientation.z = oz;
        target_pose.orientation.w = ow;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        group_arm_with_torso.setPoseTarget(target_pose);
        bool success = group_arm_with_torso.plan(my_plan);
        if (success && moveRealRobot)
        {
            group_arm_with_torso.move();
        }
        return success;
    }
    bool goToPoseGoalWithoutTorso(double ox, double oy, double oz, double ow, double x, double y, double z)
    // ox=0.013, oy=0.713, oz=0, ow=0.701, x=0.229, y=-0.063, z=0.672
    {
        geometry_msgs::Pose target_pose;
        target_pose.orientation.x = ox;
        target_pose.orientation.y = oy;
        target_pose.orientation.z = oz;
        target_pose.orientation.w = ow;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        group_arm.setPoseTarget(target_pose);
        bool success = group_arm.plan(my_plan);
        if (success && moveRealRobot)
        {
            group_arm.move();
        }
        return success;
    }

    bool goToWaypointByCartesianPathsWithTorso(double distance)
    {
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose target_pose = group_arm_with_torso.getCurrentPose().pose;
        waypoints.push_back(target_pose);
        target_pose.position.z += distance;
        waypoints.push_back(target_pose);
//        group_arm_with_torso.setPoseTarget(target_pose);
//        bool success = group_arm_with_torso.plan(my_plan);
        moveit_msgs::RobotTrajectory trajectory;
        group_arm_with_torso.setPlanningTime(10.0);

        double fraction = group_arm_with_torso.computeCartesianPath(waypoints,
                                                                    0.01,  // eef_step
                                                                    0.0,   // jump_threshold
                                                                    trajectory,
                                                                    true);
        // The trajectory needs to be modified so it will include velocities as well.
        // First to create a RobotTrajectory object
        robot_trajectory::RobotTrajectory rt(group_arm_with_torso.getCurrentState()->getRobotModel(), "arm_with_torso");
        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*group_arm_with_torso.getCurrentState(), trajectory);
        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt);
        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);
        // Finally plan and execute the trajectory
        my_plan.trajectory_ = trajectory;
        ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
        sleep(5.0);
        if (success && moveRealRobot){
            group_arm_with_torso.execute(my_plan);
        }
    }

    bool addOrientationConstrain()
    {
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "gripper_link";
        ocm.header.frame_id = "base_link";
        ocm.orientation.z = -0.707;
        ocm.orientation.x = 0.707;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;

        moveit_msgs::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);
        group_arm_with_torso.setPathConstraints(test_constraints);
        group_arm.setPathConstraints(test_constraints);
    }

    void visualPlan()
    {
        for (int i = 0; i < 2; ++i) {
            ROS_INFO("Visualizing plan 1 (again)");
            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = my_plan.start_state_;
            display_trajectory.trajectory.push_back(my_plan.trajectory_);
            display_publisher.publish(display_trajectory);
            /* Sleep to give Rviz time to visualize the plan. */
            sleep(3.0);
        }
    }

    void test()
    {
        bool a=true;
        nh_.param("move_real_robot", a, true);
        if (a)
        {
            ROS_INFO("not works");
        }
        else{
            ROS_INFO("work");
        }
    }

    void screw()
    {
        group_arm.clearPathConstraints();
//        group_arm_with_torso.clearPathConstraints();
        std::vector<double> group_variable_values;
        group_arm.getCurrentState()->copyJointGroupPositions(group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName()), group_variable_values);
        for (int i = 0; i < 900; ++i) {
            group_variable_values[7] += 0.1;
            group_arm.setJointValueTarget(group_variable_values);
            bool success = group_arm.plan(my_plan);
            if (success && moveRealRobot)
            {
                group_arm.move();
            }
        }

    }
    void screwWithTorso(int direction)
    {
//        group_arm.clearPathConstraints();
//        group_arm_with_torso.clearPathConstraints();
        std::vector<double> group_variable_values;
        group_arm_with_torso.getCurrentState()->copyJointGroupPositions(group_arm_with_torso.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_with_torso.getName()), group_variable_values);
        for (int i = 0; i < 20; ++i) {
            group_variable_values[7] -= direction* 0.1;
            group_arm_with_torso.setJointValueTarget(group_variable_values);
            bool success = group_arm_with_torso.plan(my_plan);
            if (success && moveRealRobot)
            {
                group_arm_with_torso.move();
            }
        }
    }

    tf::StampedTransform findAndGetQRcodePose() {
        tf::TransformListener listener;
        control_msgs::PointHeadGoal headGoal;
        headGoal.target.header.stamp = ros::Time::now();
        headGoal.target.header.frame_id = "base_link";
        headGoal.target.point.x = 1;
        float t=0;
        headGoal.target.point.z = 0.5;
        int count=0;

        while (nh_.ok()) {
            tf::StampedTransform transform;
            transform.setOrigin(tf::Vector3(100,1,1));
            try {
                count++;
                listener.lookupTransform("base_link", "tag_1",
                                         ros::Time(0), transform);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                std::string exString(ex.what());
//                ROS_ERROR_STREAM("test:" << exString.find("source_frame"));
                if(exString.find("source_frame")==43){
                    t += 10;
                    headGoal.target.point.y = 2 * cos(t * PI / 180.0);
                    head_action_client.sendGoal(headGoal);
                    head_action_client.waitForResult(ros::Duration(3.0));
                    if (head_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        printf("Yay! The head is reached now");
                }
                ros::Duration(2.0).sleep();
            }
            if (transform.getOrigin().x()!=100)
            {
                return transform;
            }
        }
    }



private:
    ros::NodeHandle nh_;

    moveit::planning_interface::MoveGroup group_arm;
    moveit::planning_interface::MoveGroup group_arm_with_torso;
    moveit::planning_interface::PlanningSceneInterface current_scene;
    moveit::planning_interface::MoveGroup::Plan my_plan;

    ros::Publisher display_publisher;

    bool moveRealRobot;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    FetchRobot fetchRobot(node_handle);
    ROS_INFO("Open Gripper");
    control_msgs::GripperCommandGoal grasp_pos;
    grasp_pos.command.position = 0.1;
    grasp_pos.command.max_effort = 0.0;
    fetchRobot.gripper_action_client.sendGoal(grasp_pos);

    tf::StampedTransform QrcodePose = fetchRobot.findAndGetQRcodePose();

//    ROS_INFO("Add fixture to scene");
//    fetchRobot.addFixtureToScene(0.23, 0, 0.44);
    ROS_INFO("Add table to scene");
    fetchRobot.addTableToScene();

    //Translation: [0.287, -0.143, 0.043]
    ROS_INFO("Go to top of the tube");
    ROS_INFO_STREAM("x:"<<QrcodePose.getOrigin().x()<<" y:"<<QrcodePose.getOrigin().y()<<" z:"<<QrcodePose.getOrigin().z());

    // z: 0.166 is the tf from wrist_roll_link to gripper_link
    bool test = fetchRobot.goToPoseGoalWithTorso(0.707, 0,-0.707, 0, QrcodePose.getOrigin().x()-0.060, QrcodePose.getOrigin().y()+0.142, QrcodePose.getOrigin().z()+0.166+0.302);
    ROS_INFO_STREAM("test:"<<test);
//    fetchRobot.visualPlan();
    ros::Duration(3).sleep();

    ROS_INFO("Down:");
//    fetchRobot.goToPoseGoalWithoutTorso(0.707, 0,-0.707, 0, 0.23, -0.057, 0.87);
    fetchRobot.goToWaypointByCartesianPathsWithTorso(-0.16);

    grasp_pos.command.position = 0.03;
    grasp_pos.command.max_effort = 0.0;
    fetchRobot.gripper_action_client.sendGoal(grasp_pos);
    fetchRobot.screwWithTorso(1);
    fetchRobot.goToWaypointByCartesianPathsWithTorso(0.16);

    ros::Duration(3).sleep();

    test = fetchRobot.goToPoseGoalWithTorso(0.707, 0,-0.707, 0, QrcodePose.getOrigin().x()+0.100, QrcodePose.getOrigin().y()+0.182, QrcodePose.getOrigin().z()+0.166+0.302);
    grasp_pos.command.position = 0.1;
    grasp_pos.command.max_effort = 0.0;
    fetchRobot.gripper_action_client.sendGoal(grasp_pos);


/*    ROS_INFO("Go to top of the tube");
    fetchRobot.goToPoseGoalWithTorso(0.707, 0,-0.707, 0, 0.23, -0.057, 0.8);
    fetchRobot.visualPlan();

    ROS_INFO("Down:");
    fetchRobot.goToWaypointByCartesianPathsWithTorso(-0.15);

*/


/*
    fetchRobot.test();


    ros::Duration(10).sleep();


    ROS_INFO("Go to top of the tube");
    fetchRobot.goToPoseGoalWithTorso(0.707, 0,-0.707, 0, 0.23, -0.057, 0.8);

//    fetchRobot.visualPlan();
    ros::Duration(3).sleep();
    ROS_INFO("Remove fixture to scene");
    fetchRobot.removeFixtureToScene();
    ROS_INFO("ADD CONSTRAIN:");
    fetchRobot.addOrientationConstrain();

    ROS_INFO("Down:");
//    fetchRobot.goToPoseGoalWithoutTorso(0.707, 0,-0.707, 0, 0.23, -0.057, 0.87);
    fetchRobot.goToWaypointByCartesianPathsWithTorso(-0.15);

// Grasp
//    ROS_INFO("Close gripper:");
//    grasp_pos.command.position = 0.03;
//    grasp_pos.command.max_effort = 0.0;
//    fetchRobot.gripper_action_client.sendGoal(grasp_pos);


    ros::Duration(3).sleep();
//    ROS_INFO("Screw:");
//    fetchRobot.screw();
//    fetchRobot.screwWithTorso();
    ROS_INFO("Up:");
//    fetchRobot.goToPoseGoalWithTorso(0.707, 0,-0.707, 0, 0.23, -0.057, 0.9);
    fetchRobot.goToWaypointByCartesianPathsWithTorso(0.15);
    ros::Duration(3).sleep();


// Move head
//    control_msgs::PointHeadGoal headGoal;
//    headGoal.target.header.stamp = ros::Time::now();
//    headGoal.target.header.frame_id = "base_link";
//    headGoal.target.point.x = 1;
//    headGoal.target.point.y = 1;
//    headGoal.target.point.z = 0.5;
//    fetchRobot.head_action_client.sendGoal(headGoal);


//    grasp_pos.command.position = 0.1;
//    grasp_pos.command.max_effort = 0.0;
//    fetchRobot.gripper_action_client.sendGoal(grasp_pos);
    ros::Duration(3).sleep();

*/


    ros::shutdown();
    return 0;
}
