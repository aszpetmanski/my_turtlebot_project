#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_turtle_interfaces/action/move_turtle.hpp"


using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_turtle_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

class TurtleController : public rclcpp_lifecycle::LifecycleNode {

    public:

    TurtleController() : LifecycleNode("turtle_controller") {
        this->declare_parameter("turtle_name", "turtle1");
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        RCLCPP_INFO(this->get_logger(), "Turtle controller initialized. Waiting for configuration.");
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // DOKONCZ ROBIENIE ACTION SERVER (WSZYSTKIE CALLBACKI)
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
        turtle_name_.c_str()
    );

        return LifecycleCallbackReturn::SUCCESS;

    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override {
        (void)previous_state;

        kill_client_.reset();
        spawn_client_.reset();
        move_turtle_server_.reset();
        cb_group_.reset();

        return LifecycleCallbackReturn::SUCCESS;
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal) {
        (void)uuid;
        (void)goal;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;


        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;
    }

    private:

    std::string turtle_name_;

    

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    
    rclcpp_action::Server<my_turtle_interfaces::action::MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

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