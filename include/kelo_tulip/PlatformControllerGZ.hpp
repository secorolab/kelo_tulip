#ifndef PLATFORM_CONTROLLER_GZ_HPP
#define PLATFORM_CONTROLLER_GZ_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <kelo_tulip/VelocityPlatformController.h>

/**
 * @brief Velocity controller for gazebo simulations of arbitrary
 * KELO platform configurations
 */
class PlatformControllerGZ : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new KELO Platform Controller object
     */
    PlatformControllerGZ();

    ~PlatformControllerGZ();

    /**
     * @brief Set the maximum linear and angular velocities for the KELO platform
     *
     * @param linearVel Max linear velocity the KELO platform could achieve
     * @param angularVel Max angular velocity the KELO platform could achieve
     */
    void setMaxPlatformVelocity(double linearVel, double angularVel);

    /**
     * @brief Controller step which computes and sets the desired hub wheel velocities
     */
    void step();

    /**
     * @brief Set the wheel setpoint velocities for all the hub wheels in the KELO platform
     */
    void setAllHubWheelVelocities(const std_msgs::msg::Float64MultiArray &msg);

    /**
     * @brief Callback function to receive and process the robot joint states from Gazebo
     *
     * @param msg ROS message from Gazebo with the Joint state information
     */
    void jointStatesCallBack(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Initialize the Kelo drive data structures and the Velocity controller
     */
    void initDrives();

    /**
     * @brief Set the Pivot Orientations for every Kelo drive in the KELO platform
     *
     * @param pivotJointData Map of Kelo drive name vs the latest pivot orientation
     */
    void setPivotOrientations();

    /**
     * @brief Callback function to receive and process the velocity commands for the KELO platform
     *
     * @param msg ROS message with the velocity commands for the KELO platform
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Main control loop for the KELO platform
     */
    void controlLoop();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _jointStatesSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdVelSubscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_velocity_control_pub_;
    rclcpp::TimerBase::SharedPtr _controlTimer;

    bool _initialized{ false };

    std::string base_control_mode_;
    int num_wheels_;
    std::string wheels_controller_;
    std::string pivot_joint_identifier_;

    double _cmdVelX{ 0.0 };
    double _cmdVelY{ 0.0 };
    double _cmdVelA{ 0.0 };

    struct PivotJointData
    {
        std::string jointName;
        double pivotAngle;
    };

    std::vector<PivotJointData> _pivotJointDataVec;

    sensor_msgs::msg::JointState _jointStateMsg;

    kelo::VelocityPlatformController _controller;
};

#endif// PLATFORM_CONTROLLER_HPP
