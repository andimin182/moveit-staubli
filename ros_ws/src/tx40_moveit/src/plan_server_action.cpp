#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tx40_msgs/action/plan.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <functional>
#include <memory>
#include <thread>

class PlanActionServer : public rclcpp::Node {
    public:
        using Plan = tx40_msgs::action::Plan;
        using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;

        explicit PlanActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("tx40_plan_action_server", options) {

        
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "TX40 Planning action server initialized!");

        auto handle_goal = [this](
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Plan::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Received target pose");
                (void)uuid;
                (void)goal;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                };

        auto handle_cancel = [this](
            const std::shared_ptr<GoalHandlePlan> pose_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)pose_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        auto handle_accepted = [this](
                const std::shared_ptr<GoalHandlePlan> pose_handle)
            {
                // this needs to return quickly to avoid blocking the executor,
                // so we declare a lambda function to be called inside a new thread
                auto execute_in_thread = [this, pose_handle]()
                { 
                    return this->execute(pose_handle); 
                    };
                std::thread{execute_in_thread}.detach();
            };

        this->action_server_ = rclcpp_action::create_server<Plan>(
            this,
            "tx40_move_to_pose",
            handle_goal,
            handle_cancel,
            handle_accepted);
        } // constructor

        ~PlanActionServer() {
            if(executor_ && spinner_.joinable()){
                executor_->cancel();
                spinner_.join();
            }
        } // destructor

        void initInterfaces() {
            // Create a new thread to spin the current node
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(std::shared_ptr<rclcpp::Node>(this, [](auto){}));
            spinner_ = std::thread([this]() {this->executor_->spin(); });

            // Initialize moveit group interface for the arm
            //std::shared_ptr<rclcpp::Node>(this, [](auto){})
            arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](auto){}), "arm");

            // Moveit visual tool
            // Construct and initialize MoveItVisualTools
            moveit_visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
                std::shared_ptr<rclcpp::Node>(this, [](auto){}), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                arm_group_->getRobotModel());
            moveit_visual_tools_->deleteAllMarkers();
            moveit_visual_tools_->trigger();
            moveit_visual_tools_->loadRemoteControl();
        } // init interfaces

        std::future<moveit::core::MoveItErrorCode> goToTargetPoseAsync(const geometry_msgs::msg::Pose& target_pose) {
            // Use a promise/future pair
            auto promise = std::make_shared<std::promise<moveit::core::MoveItErrorCode>>();
            auto future_result = promise->get_future();

            // Clean visuals
            moveit_visual_tools_->deleteAllMarkers();
            moveit_visual_tools_->trigger();

            // Create lambda function
            const auto execute_plan = [this, target_pose, promise]()
            {
                arm_group_->setPoseTarget(target_pose);

                // Create a plan to that target pose
                auto const [success, plan] = [this]{
                    moveit::planning_interface::MoveGroupInterface::Plan msg;
                    auto const ok = static_cast<bool>(this->arm_group_->plan(msg));
                    return std::make_pair(ok, msg);
                }();

                // Execute the plan
                if(success)
                {
                    this->draw_trajectory_path(plan);
                    moveit_visual_tools_->trigger();
                    arm_group_->execute(plan);
                    promise->set_value(moveit::core::MoveItErrorCode::SUCCESS);
                    return;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Planning to target pose failed!");
                    promise->set_value(moveit::core::MoveItErrorCode::FAILURE);
                    return;
                }
            };

            // Launch planning + execution in a background thread
            std::thread{execute_plan}
                .detach();
            return future_result;
        }

        void draw_trajectory_path(const moveit::planning_interface::MoveGroupInterface::Plan& plan){
            auto const jmg = this->arm_group_->getRobotModel()->getJointModelGroup("arm");
            auto const ee = this->arm_group_->getRobotModel()->getLinkModel("ee");
            this->moveit_visual_tools_->publishTrajectoryLine(plan.trajectory_, ee, jmg, rviz_visual_tools::BLUE);

        }

    private:
        rclcpp_action::Server<Plan>::SharedPtr action_server_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::unique_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
        std::thread spinner_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

        void execute(const std::shared_ptr<GoalHandlePlan> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Executing goal pose");
            rclcpp::Rate loop_rate(10);
            // Retrieve the goal pose from the handle
            const auto goal = goal_handle->get_goal();
            // Init a feedback and a result object
            auto feedback = std::make_shared<Plan::Feedback>();
            auto result = std::make_shared<Plan::Result>();
            auto pose_target = goal->pose_target;

            auto future_result = this->goToTargetPoseAsync(pose_target);

            while(rclcpp::ok()) {
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                // Fill the feedback message
                feedback->current_joints.header.stamp = this->now();
                feedback->current_joints.name = arm_group_->getJointNames();
                feedback->current_joints.position = arm_group_->getCurrentJointValues();               
                // Publish feedback
                RCLCPP_INFO(this->get_logger(), "Publish feedback");
                goal_handle->publish_feedback(feedback);

                // Check if goal is done
                if (future_result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    auto motion_result = future_result.get();
                    if(motion_result == moveit::core::MoveItErrorCode::SUCCESS){
                        result->success = true;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "Goal pose reached with success!");
                        break;
                    }else if (motion_result == moveit::core::MoveItErrorCode::FAILURE){
                        result->success = false;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "Error with pose target!");
                        break;
                    }
                }; // if loop
                loop_rate.sleep();
            }; // while loop
        } // execute method

}; // class Plan Action Server

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto plan_action_server = std::make_shared<PlanActionServer>();
    plan_action_server->initInterfaces();
    // Keep main thread alive while background spinner runs
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();
    return 0;
}