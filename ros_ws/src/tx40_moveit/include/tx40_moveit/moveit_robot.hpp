#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <moveit_visual_tools/moveit_visual_tools.h>

class MoveItRobot {
    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::thread spinner_;
        std::unique_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

    public:
        // Constructor
        MoveItRobot();

        // Getter for node
        std::shared_ptr<rclcpp::Node> getNode() const;

        void init();
        void shutdown();

        void goToTargetPose(const geometry_msgs::msg::Pose &target_pose);
        void goToTargetJoint(const std::vector<double> &target_joints);
        void createCollisionObject();
};