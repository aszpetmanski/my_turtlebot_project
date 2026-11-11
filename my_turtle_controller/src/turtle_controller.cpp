#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_turtle_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <mutex>



using KillTurtle = turtlesim::srv::Kill;
using SpawnTurtle = turtlesim::srv::Spawn;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_turtle_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using namespace std::chrono_literals;

class TurtleController : public rclcpp_lifecycle::LifecycleNode {

    public:

    TurtleController() : LifecycleNode("turtle_controller") {
        this->declare_parameter("turtle_name", "alan");
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        action_server_activated_ = false;
        RCLCPP_INFO(this->get_logger(), "Turtle controller initialized. Waiting for configuration.");
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + turtle_name_ + "/cmd_vel", 10);

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
                                                            this, "move_turtle_" + turtle_name_, 
                                                            [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal){return handle_goal(uuid, goal);},
                                                            [this](const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){return handle_cancel(goal_handle);},
                                                            [this](const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){handle_accepted(goal_handle);},
                                                        rcl_action_server_get_default_options(),
                                                        cb_group_);
        RCLCPP_INFO(
        this->get_logger(),
        "Turtle controller configured for '%s'. Action server: 'move_turtle_%s'.",
        turtle_name_.c_str(),
        turtle_name_.c_str());
        
        // Kill turtle
        kill_turtle();
        spawn_turtle();

        // Spawn turtle

        return LifecycleCallbackReturn::SUCCESS;

    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        kill_client_.reset();
        spawn_client_.reset();
        cmd_vel_publisher_.reset();
        move_turtle_server_.reset();
        cb_group_.reset();

        action_server_activated_ = false;
        //TO DO WYJEB WSZYSTKO CO STWORZYLES

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        action_server_activated_ = true;
        RCLCPP_INFO(this->get_logger(), "Node activated.");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        action_server_activated_ = false;

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        kill_client_.reset();
        spawn_client_.reset();
        cmd_vel_publisher_.reset();
        move_turtle_server_.reset();
        cb_group_.reset();

        action_server_activated_ = false;
        

        return LifecycleCallbackReturn::SUCCESS;
    }

    void kill_turtle() {
        kill_client_->wait_for_service();

        auto request = std::make_shared<KillTurtle::Request>();
        request->name = "turtle1";

        kill_client_->async_send_request(request);
    }

    void spawn_turtle() {
        spawn_client_->wait_for_service();

        auto request = std::make_shared<SpawnTurtle::Request>();
        request->name = turtle_name_;
        request->x = 5.54;
        request->y = 5.54;
        
        spawn_client_->async_send_request(request);
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal) {
        (void)uuid;
        (void)goal;

        if (!action_server_activated_) {
            RCLCPP_WARN(this->get_logger(), "Server not active yet");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // TO DO ODRZUC GOAL JAK MASZ AKTYWNY

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (this->goal_handle_ && this->goal_handle_->is_active()) {
                RCLCPP_WARN(this->get_logger(), "Rejecting the goal because another is active");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Acepting the goal.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }
        RCLCPP_INFO(this->get_logger(), "Calling execute!");
        execute(goal_handle);
    }

    void execute(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {

        auto feedback = std::make_shared<MoveTurtle::Feedback>();
        auto result = std::make_shared<MoveTurtle::Result>();

        geometry_msgs::msg::Twist cmd_vel_msg;
        
        int action_duration = goal_handle->get_goal()->duration_s;
        float linear_x = goal_handle->get_goal()->linear_vel_x;
        float angular_z = goal_handle->get_goal()->angular_vel_z;

        rclcpp::Duration duration = rclcpp::Duration::from_seconds(action_duration);
        rclcpp::Time start_time = this->now();
        rclcpp::Rate rate(30.0);
        
        RCLCPP_INFO(this->get_logger(), "Start executing the goal.");

        while (rclcpp::ok()) {

            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_WARN(this->get_logger(), "Canceled the goal.");
                return;
            }


            if ((this->now() - start_time) >= duration) {

                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd_vel_msg);

                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                return;
            }

            cmd_vel_msg.linear.x = linear_x; 
            cmd_vel_msg.angular.z = angular_z;
            cmd_vel_publisher_->publish(cmd_vel_msg);

            rate.sleep();

        }


    }



    private:

    std::string turtle_name_;
    bool action_server_activated_;
    std::mutex mutex_;
    

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    rclcpp_action::Server<my_turtle_interfaces::action::MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;

};


int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TurtleController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

