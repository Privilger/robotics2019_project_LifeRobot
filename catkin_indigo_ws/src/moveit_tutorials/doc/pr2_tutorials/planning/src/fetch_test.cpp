#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <robot_calibration_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

class FetchRobot
{
public:
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;
    actionlib::SimpleActionClient<robot_calibration_msgs::GripperLedCommandAction> led_action_client;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head_action_client;

    FetchRobot(ros::NodeHandle& nodehandle) :
    moveRealRobot(true),
    nh_(nodehandle),
    gripper_action_client("/gripper_controller/gripper_action",true),
    led_action_client("/gripper_controller/led_action",true),
    head_action_client("/head_controller/point_head",true),
    group_arm("arm"),
    group_arm_with_torso("arm_with_torso")
    {
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
//        sleep(5.0);
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

    bool goTo

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
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    FetchRobot fetchRobot(node_handle);
//    fetchRobot.addFixtureToScene(0.23, 0, 0.44);
//    ros::Duration(10).sleep();
//    fetchRobot.removeFixtureToScene();

// Grasp
//    control_msgs::GripperCommandGoal grasp_pos;
//    grasp_pos.command.position = 0.03;
//    grasp_pos.command.max_effort = 0.0;
//    fetchRobot.gripper_action_client.sendGoal(grasp_pos);

// Move head
//    control_msgs::PointHeadGoal headGoal;
//    headGoal.target.header.stamp = ros::Time::now();
//    headGoal.target.header.frame_id = "base_link";
//    headGoal.target.point.x = 1;
//    headGoal.target.point.y = 1;
//    headGoal.target.point.z = 0.5;
//    fetchRobot.head_action_client.sendGoal(headGoal);





    ros::shutdown();
  return 0;
}
