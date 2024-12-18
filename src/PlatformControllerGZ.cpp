#include "kelo_tulip/PlatformControllerGZ.hpp"

PlatformControllerGZ::PlatformControllerGZ()
  : Node("kelo_platform_controller_gz"), _cmdVelX(0.0), _cmdVelY(0.0), _cmdVelA(0.0),
    _initialized(false)
{
    // parameters
    this->declare_parameter("base_control_mode", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("wheels_controller", rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("num_wheels", rclcpp::ParameterType::PARAMETER_INTEGER);
    this->declare_parameter("platform_max_lin_vel", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("platform_max_ang_vel", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("pivot_joint_identifier", rclcpp::ParameterType::PARAMETER_STRING);

    num_wheels_ = this->get_parameter("num_wheels").as_int();
    base_control_mode_ = this->get_parameter("base_control_mode").as_string();
    wheels_controller_ = this->get_parameter("wheels_controller").as_string();
    pivot_joint_identifier_ = this->get_parameter("pivot_joint_identifier").as_string();

    for (int i = 0; i < num_wheels_; i++) {
        this->declare_parameter(
          "wheels.wheel" + std::to_string(i) + ".x", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter(
          "wheels.wheel" + std::to_string(i) + ".y", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter(
          "wheels.wheel" + std::to_string(i) + ".a", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("wheels.wheel" + std::to_string(i) + ".identifier",
          rclcpp::ParameterType::PARAMETER_STRING);
    }

    // set max platform velocity
    this->setMaxPlatformVelocity(this->get_parameter("platform_max_lin_vel").as_double(),
      this->get_parameter("platform_max_ang_vel").as_double());

    // Setup subscribers
    _jointStatesSubscriber =
      this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
        rclcpp::QoS(10),
        std::bind(&PlatformControllerGZ::jointStatesCallBack, this, std::placeholders::_1));

    // Initialize drives
    this->initDrives();

    // Setup publishers
    if (base_control_mode_ == "VELOCITY") {
        base_velocity_control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/" + wheels_controller_ + "/commands", 10);

        _cmdVelSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",
          10,
          std::bind(&PlatformControllerGZ::cmdVelCallback, this, std::placeholders::_1));
    }

    // timer
    _controlTimer = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&PlatformControllerGZ::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "PlatformControllerGZ initialized");
}

PlatformControllerGZ::~PlatformControllerGZ() {}

void PlatformControllerGZ::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    _cmdVelX = msg->linear.x;
    _cmdVelY = msg->linear.y;
    _cmdVelA = msg->angular.z;
}

void PlatformControllerGZ::controlLoop() { this->step(); }

void PlatformControllerGZ::setMaxPlatformVelocity(double linearVel, double angularVel)
{
    _controller.setPlatformMaxLinVelocity(linearVel);
    _controller.setPlatformMaxAngVelocity(angularVel);
}

void PlatformControllerGZ::step()
{
    if (_initialized) {
        _controller.setPlatformTargetVelocity(_cmdVelX, _cmdVelY, _cmdVelA);
        _controller.calculatePlatformRampedVelocities();
        std_msgs::msg::Float64MultiArray message;
        message.data.resize(num_wheels_ * 2);

        for (int i = 0; i < num_wheels_; i++) {
            float left_whl_sp, right_whl_sp;
            _controller.calculateWheelTargetVelocity(
              i, _pivotJointDataVec[i].pivotAngle, left_whl_sp, right_whl_sp);

            auto wheel_identifier =
              this->get_parameter("wheels.wheel" + std::to_string(i) + ".identifier").as_string();

            message.data[2 * i] = left_whl_sp;
            message.data[2 * i + 1] = -right_whl_sp;
        }

        setAllHubWheelVelocities(message);
    }
}

void PlatformControllerGZ::initDrives()
{
    std::vector<kelo::WheelConfig> wheelConfigsVector;
    double zDummy = 0.0;
    for (int i = 0; i < num_wheels_; i++) {
        kelo::WheelConfig wc;
        wc.ethercatNumber = i;
        wc.x = this->get_parameter("wheels.wheel" + std::to_string(i) + ".x").as_double();
        wc.y = this->get_parameter("wheels.wheel" + std::to_string(i) + ".y").as_double();
        wc.a = this->get_parameter("wheels.wheel" + std::to_string(i) + ".a").as_double();
        wheelConfigsVector.push_back(wc);
    }
    _controller.initialise(wheelConfigsVector);

    // wait for the first joint states message to arrive
    while (_jointStateMsg.name.size() == 0) { rclcpp::spin_some(this->get_node_base_interface()); }

    for (int i = 0; i < num_wheels_; i++) {
        auto wheel_identifier =
          this->get_parameter("wheels.wheel" + std::to_string(i) + ".identifier").as_string();
        // find the joint name that contains the wheel identifier and pivot joint identifier
        for (size_t j = 0; j < _jointStateMsg.name.size(); ++j) {
            if (_jointStateMsg.name[j].find(wheel_identifier) != std::string::npos
                && _jointStateMsg.name[j].find(pivot_joint_identifier_) != std::string::npos) {
                PivotJointData pivotJointData;
                pivotJointData.jointName = _jointStateMsg.name[j];
                double pivotAngle = _jointStateMsg.position[j];
                // convert pivot angle to range [0, 2*PI)
                pivotAngle -= int(pivotAngle / (2 * M_PI)) * 2 * M_PI;
                pivotJointData.pivotAngle = pivotAngle;
                _pivotJointDataVec.push_back(pivotJointData);
                break;
            }
        }
    }

    if (_pivotJointDataVec.size() != num_wheels_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize pivot joint data");
        return;
    }

    _initialized = true;
    RCLCPP_INFO(this->get_logger(), "Initialized Kelo drives");
}

void PlatformControllerGZ::jointStatesCallBack(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    _jointStateMsg = *msg;

    if (_initialized) this->setPivotOrientations();
}

void PlatformControllerGZ::setPivotOrientations()
{
    for (int i = 0; i < num_wheels_; i++) {
        auto joint_name = _pivotJointDataVec[i].jointName;
        // get the corresponding joint position from the joint state message
        auto joint_position = _jointStateMsg.position[std::distance(_jointStateMsg.name.begin(),
          std::find(_jointStateMsg.name.begin(), _jointStateMsg.name.end(), joint_name))];
        // convert pivot angle to range [0, 2*PI)
        joint_position -= int(joint_position / (2 * M_PI)) * 2 * M_PI;
        _pivotJointDataVec[i].pivotAngle = joint_position;
    }
}

void PlatformControllerGZ::setAllHubWheelVelocities(const std_msgs::msg::Float64MultiArray &msg)
{
    if (!base_velocity_control_pub_) {
        RCLCPP_WARN(this->get_logger(), "Publisher is not initialized!");
        return;
    }
    base_velocity_control_pub_->publish(msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformControllerGZ>());
    rclcpp::shutdown();
    return 0;
}