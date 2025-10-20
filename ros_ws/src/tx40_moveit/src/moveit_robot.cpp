#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tx40_moveit/moveit_robot.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

MoveItRobot::MoveItRobot() : node_(std::make_shared<rclcpp::Node>(
                                    "manipulator_moveit",
                                    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
                                    )) {
                RCLCPP_INFO(node_->get_logger(), "MoveItRobot initialized!");
            };

std::shared_ptr<rclcpp::Node> MoveItRobot::getNode() const {
    return node_;
}

void MoveItRobot::init() {
    // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
    rclcpp::executors::SingleThreadedExecutor *executor = new rclcpp::executors::SingleThreadedExecutor;
    executor->add_node(node_);
    spinner_ = std::thread([executor]() { executor->spin(); });
    // Create the moveit arm group interface
    arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface for arm created!");

    // Moveit visual tool
    // Construct and initialize MoveItVisualTools
    moveit_visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
        node_, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        arm_group_->getRobotModel());
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->trigger();
    moveit_visual_tools_->loadRemoteControl();
}

void MoveItRobot::shutdown() {
    // Shutdown ROS
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    spinner_.join();  // <--- Join the thread before exiting
}

void MoveItRobot::goToTargetJoint(const std::vector<double> &target_joints) {
    // Create closures for visualization
    auto const draw_title = [this](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;  // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
    };
    auto const prompt = [this](auto text) {
    this->moveit_visual_tools_->prompt(text);
    };
    auto const draw_trajectory_tool_path =
        [this,
        jmg = arm_group_->getRobotModel()->getJointModelGroup(
            "arm"),
        ee = arm_group_->getRobotModel()->getLinkModel("ee")](auto const trajectory) {
        this->moveit_visual_tools_->publishTrajectoryLine(trajectory,ee, jmg, rviz_visual_tools::BLUE);
        };

    bool arm_within_lim = arm_group_->setJointValueTarget(target_joints);
    if(!arm_within_lim){
        RCLCPP_WARN(node_->get_logger(), "Target joint positions are outside the workspace");
        return;
    }
    // Add object into scene
    this->createCollisionObject();

    // Create a plan to that target pose
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    moveit_visual_tools_->trigger();
    auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->arm_group_->plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools_->trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools_->trigger();
        arm_group_->execute(plan);
    } else {
        draw_title("Planning Failed!");
        moveit_visual_tools_->trigger();
        RCLCPP_ERROR(node_->get_logger(), "Planning to target joints failed!");
    }
    //moveit_visual_tools_->deleteAllMarkers();
    //moveit_visual_tools_->trigger();
}

void MoveItRobot::goToTargetPose(const geometry_msgs::msg::Pose &target_pose) {
    RCLCPP_INFO(node_->get_logger(), "Received target pose position = [%.3f, %.3f, %.3f], orientation = [%.3f, %.3f, %.3f, %.3f]",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);

    arm_group_->setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [this]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(this->arm_group_->plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        arm_group_->execute(plan);
    } else {
    RCLCPP_ERROR(node_->get_logger(), "Planning to target pose failed!");
    }
}

void MoveItRobot::createCollisionObject() {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = arm_group_->getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  RCLCPP_INFO(node_->get_logger(), "Collision object correctly added!");

}