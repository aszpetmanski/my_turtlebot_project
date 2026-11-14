#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_turtle_interfaces/action/move_turtle.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <mutex>
#include <cmath>



using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MoveTurtle = my_turtle_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using namespace std::chrono_literals;


namespace my_turtle_controller_gz {

class TurtleController : public rclcpp_lifecycle::LifecycleNode {

    public:

    TurtleController(const rclcpp::NodeOptions &options);

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    void stop_turtle();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveTurtle::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);

    void execute(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);



    private:

    bool action_server_activated_;
    bool goal_received_;
    std::mutex mutex_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
    rclcpp_action::Server<my_turtle_interfaces::action::MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;

};

} //namespace my_turtle_controller_gz