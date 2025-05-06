#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class MoveItPlannerNode : public rclcpp::Node {
public:
    MoveItPlannerNode() : Node("moveit_planner_node") {
        // Set up MoveIt MoveGroup interface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");

        // Publisher for current pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/robot/current_pose", 10);

        // Timer to periodically publish robot's current pose
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MoveItPlannerNode::publishCurrentPose, this)
        );

        RCLCPP_INFO(this->get_logger(), "MoveIt Planner Node initialized.");
    }

    void moveToPose(const geometry_msgs::msg::Pose &target_pose) {
        move_group_->setPoseTarget(target_pose);

        // Plan and execute
        bool success = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Move successful!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Move failed!");
        }

        // Clear pose targets
        move_group_->clearPoseTargets();
    }

    void returnToHome() {
        move_group_->setNamedTarget("home");
        move_group_->move();
        RCLCPP_INFO(this->get_logger(), "Returned to home position.");
    }

private:
    void publishCurrentPose() {
        geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
        pose_publisher_->publish(current_pose);
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItPlannerNode>();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.5;
    target_pose.orientation.w = 1.0;

    node->moveToPose(target_pose);
    node->returnToHome();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
