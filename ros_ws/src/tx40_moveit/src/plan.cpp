#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tx40_moveit/moveit_robot.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto manipulator = MoveItRobot();
  manipulator.init();

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.06;
    msg.orientation.y = 0.6;
    msg.orientation.z = 0.6;
    msg.orientation.w = 1;
    msg.position.x = -0.035;
    msg.position.y = 0.03;
    msg.position.z = 0.825;
    return msg;
  }();

  // Set a joint target
  const std::vector<double> target_joints = {0.0, 1.4, 0.4, 2.3, 0.4, 1.3};

  manipulator.goToTargetPose(target_pose);
  //manipulator.goToTargetJoint(target_joints);

  // Shutdown ROS
  manipulator.shutdown();
  return 0;
}