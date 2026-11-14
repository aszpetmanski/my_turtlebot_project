/*
 * TurtleController is a ROS 2 lifecycle node that manages a turtle model in turtlesim.
 * During configuration, it creates the services, velocity publisher, and
 * MoveTurtle action server, and requests the turtle to be spawned. Accepted
 * goals command linear and angular velocity for a specified duration while
 * reporting progress. The controller also handles cancellation, deactivation,
 * cleanup, and shutdown, stopping or removing the turtle as appropriate.
 */

#include "my_turtle_controller/turtle_controller_gz.hpp"



namespace my_turtle_controller_gz {

TurtleController::TurtleController(const rclcpp::NodeOptions &options)
    : LifecycleNode("turtle_controller", options) {

    // The first flag gates action requests by lifecycle state; the second
    // prevents overlapping goals, including goals still awaiting execution.
    action_server_activated_ = false;
    goal_received_ = false;

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    RCLCPP_INFO(get_logger(), "Turtle controller initialized. Waiting for configuration.");
}

LifecycleCallbackReturn TurtleController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
        this, "move_turtle",
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const MoveTurtle::Goal> goal) {
            return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
            return handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
            handle_accepted(goal_handle);
        },
        rcl_action_server_get_default_options(), cb_group_);

    RCLCPP_INFO(get_logger(),
                "Turtle controller configured. Action server: /move_turtle");
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    cmd_vel_publisher_.reset();
    move_turtle_server_.reset();

    action_server_activated_ = false;
    goal_received_ = false;
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    action_server_activated_ = true;
    RCLCPP_INFO(get_logger(), "Turtle controller activated.");
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    action_server_activated_ = false;
    goal_received_ = false;
    RCLCPP_INFO(get_logger(), "Turtle controller deactivated.");
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    cmd_vel_publisher_.reset();
    move_turtle_server_.reset();

    action_server_activated_ = false;
    goal_received_ = false;
    return LifecycleCallbackReturn::SUCCESS;
}



void TurtleController::stop_turtle() {
    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    cmd_vel_msg.twist.linear.x = 0.0;
    cmd_vel_msg.twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_msg);
}

rclcpp_action::GoalResponse TurtleController::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveTurtle::Goal> goal) {
    (void)uuid;

    if (!action_server_activated_) {
        RCLCPP_WARN(get_logger(), "Rejecting goal: action server is inactive.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_received_) {
            RCLCPP_WARN(get_logger(), "Rejecting goal: another goal is being processed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal_handle_ && goal_handle_->is_active()) {
            RCLCPP_WARN(get_logger(), "Rejecting goal: another goal is active.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        goal_received_ = true;
    }

    if (goal->duration_s <= 0 || std::fabs(goal->linear_vel_x) > 3.0 ||
        std::fabs(goal->angular_vel_z) > 3.0) {
        RCLCPP_WARN(get_logger(), "Rejecting goal: invalid duration or velocity.");
        goal_received_ = false;
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtleController::handle_cancel(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Goal cancellation requested.");
    stop_turtle();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleController::handle_accepted(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_handle_ = goal_handle;
    }
    RCLCPP_INFO(get_logger(), "Starting accepted goal execution.");
    execute(goal_handle);
}

void TurtleController::execute(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle) {
    auto feedback = std::make_shared<MoveTurtle::Feedback>();
    auto result = std::make_shared<MoveTurtle::Result>();

    geometry_msgs::msg::TwistStamped cmd_vel_msg;
    int action_duration = goal_handle->get_goal()->duration_s;
    float linear_x = goal_handle->get_goal()->linear_vel_x;
    float angular_z = goal_handle->get_goal()->angular_vel_z;

    rclcpp::Duration duration = rclcpp::Duration::from_seconds(action_duration);
    rclcpp::Time start_time = now();
    rclcpp::Rate rate(30.0);
    RCLCPP_INFO(get_logger(), "Executing goal.");

    // At 30 Hz, check for cancellation, completion, or deactivation before
    // publishing the requested velocity. Report estimated distance traveled
    // and remaining time to the action client on each iteration.


    //TO DO PRZEROB DISTANCE COVERED ZEBY BRALO Z ODOM
    while (rclcpp::ok()) {
        auto elapsed_time = now() - start_time;

        if (goal_handle->is_canceling()) {
            stop_turtle();
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled.");
            goal_received_ = false;
            return;
        }

        if (elapsed_time >= duration) {
            stop_turtle();
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal completed successfully.");
            goal_received_ = false;
            return;
        }

        if (!action_server_activated_) {
            stop_turtle();
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_WARN(get_logger(), "Goal aborted: action server deactivated.");
            goal_received_ = false;
            return;
        }

        cmd_vel_msg.twist.linear.x = linear_x;
        cmd_vel_msg.twist.angular.z = angular_z;
        cmd_vel_publisher_->publish(cmd_vel_msg);

        feedback->distance_covered = std::fabs(linear_x) * elapsed_time.seconds();
        feedback->time_left = duration.seconds() - elapsed_time.seconds();
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }
}

}  // namespace my_turtle_controller_gz

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_turtle_controller_gz::TurtleController);
