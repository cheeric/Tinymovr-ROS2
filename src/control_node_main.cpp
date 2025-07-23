#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a generic node to host the controller manager.
    auto node = std::make_shared<rclcpp::Node>("tinymovr_control_node");
    RCLCPP_INFO(node->get_logger(), "Tinymovr control node started.");

    // Use a multi-threaded executor to allow the controller manager to run its own loop.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Create the controller manager, which will load and manage the hardware interface
    // plugins and the controllers themselves.
    auto controller_manager = std::make_shared<controller_manager::ControllerManager>(node, executor);
    RCLCPP_INFO(node->get_logger(), "Controller Manager created.");
    
    // The executor's spin() call will block and manage all the callbacks,
    // including the controller manager's update loop.
    executor.spin();

    rclcpp::shutdown();
    return 0;
}