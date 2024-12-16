#ifndef PLATFORM_CONTROLLER_GZ_HPP
#define PLATFORM_CONTROLLER_GZ_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "tf2_ros/buffer.h"

#include <kelo_tulip/VelocityPlatformController.h>
#include <kelo_tulip/KeloDrive.h>

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
     * @brief Set the linear and angular velocities for the KELO platform
     *
     * @param vx Linear velocity (in m/s) along the positive x axis in robot frame
     * @param vy Linear velocity (in m/s) along the positive y axis in robot frame
     * @param va Angular velocity in (rad/s) around the positive z axis in robot frame
     */
    void setCmdVel(double vx, double vy, double va);

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
     * @brief Publish RViz markers for each active wheel's pivot pose
     */
    void publishPivotMarkers() const;

    /**
     * @brief Set the frequency at which odometry messages are published
     *
     * @param frequency Frequency (in Hz) at which odometry messages should be published
     */
    void setOdomFrequency(double frequency);

    /**
     * @brief Publish the latest odometry received from Gazebo on '/odom' topic
     */
    void publishOdom();

    /**
     * @brief Publish the latest transform from odom to base_link on the '/tf' topic
     */
    void publishOdomToBaseLinkTF();

    /**
     * @brief Set the wheel setpoint velocities for all the hub wheels in the KELO platform
     */
    void setAllHubWheelVelocities(const std_msgs::msg::Float64MultiArray &msg);


  protected:
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
     * @brief Extract the pivot name from the pivot joint name
     *
     * @param jointName Joint name of the Kelo drive pivot
     * @return std::string Name of the pivot
     */
    std::string getPivotName(const std::string &jointName);

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void controlLoop();

    // ROS2 Specific Members
    std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> _tfListener;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _jointStatesSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmdVelSubscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_velocity_control_pub_;
    rclcpp::TimerBase::SharedPtr _controlTimer;

    // Existing Members
    std::map<std::string, KeloDrive> _drives;
    std::vector<KeloDrive> _drivesVector;
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

    std::map<std::string, int> drives_map = {
        // Note: the order of the drives should be same as the order of controllers in
        // eddie_controllers.yaml (eddie_gazebo)
        // Assumption: for each wheel unit, first the left wheel and then the right wheel is
        // considered in the controller configuration file

        { "eddie_front_left", 0 },
        { "eddie_rear_left", 1 },
        { "eddie_rear_right", 2 },
        { "eddie_front_right", 3 }
    };

    kelo::VelocityPlatformController _controller;
    std::map<std::string, kelo::WheelConfig> _wheelConfigs;
};

#endif// PLATFORM_CONTROLLER_HPP
