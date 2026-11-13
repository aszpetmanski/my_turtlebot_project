#include "my_turtle_controller/turtle_controller.hpp"

namespace my_turtle_controller {


    
    TurtleController::TurtleController(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_controller", options) {
        
        this->declare_parameter("turtle_name", "alan");
        action_server_activated_ = false;
        goal_received_ = false;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        RCLCPP_INFO(this->get_logger(), "Turtle controller initialized. Waiting for configuration.");
    };
    
    LifecycleCallbackReturn TurtleController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill", rclcpp::ServicesQoS(), cb_group_);
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn", rclcpp::ServicesQoS(), cb_group_);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + turtle_name_ + "/cmd_vel", 10);
        move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
                                                            this, "move_turtle_" + turtle_name_, 
                                                            [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal){return handle_goal(uuid, goal);},
                                                            [this](const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){return handle_cancel(goal_handle);},
                                                            [this](const std::shared_ptr<MoveTurtleGoalHandle> goal_handle){handle_accepted(goal_handle);},
                                                        rcl_action_server_get_default_options(),
                                                        cb_group_);

        spawn_turtle();
        
        RCLCPP_INFO(
        this->get_logger(),
        "Turtle controller configured for '%s'. Action server: 'move_turtle_%s'.",
        turtle_name_.c_str(),
        turtle_name_.c_str());
        
        return LifecycleCallbackReturn::SUCCESS;
        
    }
    
    LifecycleCallbackReturn TurtleController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        
        kill_turtle();

        kill_client_.reset();
        spawn_client_.reset();
        cmd_vel_publisher_.reset();
        move_turtle_server_.reset();

        action_server_activated_ = false;
        goal_received_ = false;

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        action_server_activated_ = true;
        RCLCPP_INFO(this->get_logger(), "Node activated.");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        action_server_activated_ = false;
        goal_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Node deactivated.");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        kill_turtle();

        kill_client_.reset();
        spawn_client_.reset();
        cmd_vel_publisher_.reset();
        move_turtle_server_.reset();

        action_server_activated_ = false;
        goal_received_ = false;
        

        return LifecycleCallbackReturn::SUCCESS;
    }


    void TurtleController::kill_turtle() {
        kill_client_->wait_for_service();

        auto request = std::make_shared<KillTurtle::Request>();
        request->name = turtle_name_;
        RCLCPP_INFO(this->get_logger(), "Trying to kill the turtle.");
        kill_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Turtle %s has been killed", turtle_name_.c_str());
    }

    void TurtleController::spawn_turtle() {
        spawn_client_->wait_for_service();

        auto request = std::make_shared<SpawnTurtle::Request>();
        request->name = turtle_name_;
        request->x = 5.54;
        request->y = 5.54;
        RCLCPP_INFO(this->get_logger(), "Trying to spawn the turtle.");
        spawn_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Turtle %s has been spawned at x= %.2f, y= %.2f", turtle_name_.c_str(), 5.54, 5.54);
    }

    void TurtleController::stop_turtle() {

        geometry_msgs::msg::Twist cmd_vel_msg;

        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    rclcpp_action::GoalResponse TurtleController::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal) {
        (void)uuid;
        (void)goal;

        if (!action_server_activated_) {
            RCLCPP_WARN(this->get_logger(), "Server not active yet");
            return rclcpp_action::GoalResponse::REJECT;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            
            if (goal_received_) {
                RCLCPP_WARN(this->get_logger(), "Rejecting the goal because another goal is beging processed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            
            if (this->goal_handle_ && this->goal_handle_->is_active()) {
                RCLCPP_WARN(this->get_logger(), "Rejecting the goal because another is active");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            goal_received_ = true;
        }

        if (goal->duration_s <= 0 || std::fabs(goal->linear_vel_x) > 3.0 || std::fabs(goal->angular_vel_z) > 3.0) {
                RCLCPP_WARN(this->get_logger(), "Invalid goal");
                goal_received_ = false;
                return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Acepting the goal.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse TurtleController::handle_cancel(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;
        
        RCLCPP_WARN(this->get_logger(), "Received cancel request");
        stop_turtle();
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TurtleController::handle_accepted(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
        (void)goal_handle;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }
        RCLCPP_INFO(this->get_logger(), "Calling execute!");
        execute(goal_handle);
    }

    void TurtleController::execute(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {

        // dodaj feedback


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
            
            auto elapsed_time = this->now() - start_time;

            if (goal_handle->is_canceling()) {

                stop_turtle();
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_WARN(this->get_logger(), "Canceled the goal.");
                goal_received_ = false;

                return;
            }


            if (elapsed_time >= duration) {

                stop_turtle();
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                goal_received_ = false;

                return;
            }

            if (!action_server_activated_) {
                stop_turtle();
                result->success = false;
                goal_handle->abort(result);
                RCLCPP_WARN(this->get_logger(), "Server has been deactivated");
                goal_received_ = false;

                return;
            }


            cmd_vel_msg.linear.x = linear_x; 
            cmd_vel_msg.angular.z = angular_z;
            cmd_vel_publisher_->publish(cmd_vel_msg);


            feedback->distance_covered = linear_x * elapsed_time.seconds();
            feedback->time_left = duration.seconds() - elapsed_time.seconds();
            goal_handle->publish_feedback(feedback);

            rate.sleep();

        }


    }

} //namespace my_turtle_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_turtle_controller::TurtleController);


