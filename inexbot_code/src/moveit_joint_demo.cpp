//moveit_joint_demo.c
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    //Ros initialization
    ros::init(argc, argv, "moveit_joint_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //manipulator is a planning group set by moveit
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //Tolerance
    arm.setGoalJointTolerance(0.001);
    //Max ccceleration
    arm.setMaxAccelerationScalingFactor(0.2);
    //Max speed
    arm.setMaxVelocityScalingFactor(0.2);

    //Set the goal_state to control the robot arm to return to the initialization position first
    //home is the preset position of moveit
    arm.setNamedTarget("home");
    //Run
    arm.move();
    sleep(1);
    //Set the angle of the six axes of the joint space
    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1);

    // Control the robot arm to return to the initial position first
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown();

    return 0;
}
