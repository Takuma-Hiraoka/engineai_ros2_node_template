#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "interface_protocol/msg/gamepad_keys.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "interface_protocol/msg/parallel_parser_type.hpp"

namespace engineai_ros2_node_template{
  class EngineAIRos2NodeTemplate : public rclcpp::Node {
  public:
    EngineAIRos2NodeTemplate() : Node("engineai_ros2_node_template") {
      joint_command_ = std::make_shared<interface_protocol::msg::JointCommand>();
    }

    void UpdateState() {
      q_real_ = Eigen::Map<const Eigen::VectorXd>(latest_joint_state_->position.data(), latest_joint_state_->position.size());
      qd_real_ = Eigen::Map<const Eigen::VectorXd>(latest_joint_state_->velocity.data(), latest_joint_state_->velocity.size());
      quat_real_ = Eigen::Quaterniond(latest_imu_->quaternion.w, latest_imu_->quaternion.x, latest_imu_->quaternion.y, latest_imu_->quaternion.z).toRotationMatrix();
      w_real_ = Eigen::Vector3d(latest_imu_->angular_velocity.x, latest_imu_->angular_velocity.y, latest_imu_->angular_velocity.z);
    }

    void CalculateObservation() {
    }

    void CalculateMotorCommand() {
      // TODO
      q_des_ = q_real_;
      joint_kp_ = Eigen::VectorXd::Zero(q_des_.size());
      joint_kd_ = Eigen::VectorXd::Zero(q_des_.size());
    }

    void SendMotorCommand() {
      joint_command_->position = std::vector<double>(q_des_.data(), q_des_.data() + q_des_.size());
      joint_command_->velocity = std::vector<double>(q_des_.size(), 0.0);
      joint_command_->feed_forward_torque = std::vector<double>(q_des_.size(), 0.0);
      joint_command_->torque = std::vector<double>(q_des_.size(), 0.0);
      joint_command_->stiffness = std::vector<double>(joint_kp_.data(), joint_kp_.data() + joint_kp_.size());
      joint_command_->damping = std::vector<double>(joint_kd_.data(), joint_kd_.data() + joint_kd_.size());
      joint_command_->parallel_parser_type = interface_protocol::msg::ParallelParserType::RL_PARSER;
      joint_cmd_pub_->publish(*joint_command_);
    }

    void ControlCallback() {
      if (!latest_joint_state_ || !latest_imu_) return;

      UpdateState();
      CalculateObservation();
      CalculateMotorCommand();
      SendMotorCommand();
    }

    bool Initialize() {
      rclcpp::QoS qos(3);
      qos.best_effort();
      qos.durability_volatile();
      // Initialize subscribers
      gamepad_sub_ = this->create_subscription<interface_protocol::msg::GamepadKeys>("/hardware/gamepad_keys", qos, [this](const interface_protocol::msg::GamepadKeys::SharedPtr msg) {latest_gamepad_ = msg;});
      imu_sub_ = this->create_subscription<interface_protocol::msg::ImuInfo>("/hardware/imu_info", qos, [this](const interface_protocol::msg::ImuInfo::SharedPtr msg) { latest_imu_ = msg;});
      joint_state_sub_ = this->create_subscription<interface_protocol::msg::JointState>("/hardware/joint_state", qos, [this](const interface_protocol::msg::JointState::SharedPtr msg) {latest_joint_state_ = msg;});
      motion_state_sub_ = this->create_subscription<interface_protocol::msg::MotionState>("/motion/motion_state", 1, [this](const interface_protocol::msg::MotionState::SharedPtr msg) {latest_motion_state_ = msg;});
      // Initialize publisher
      joint_cmd_pub_ = this->create_publisher<interface_protocol::msg::JointCommand>("/hardware/joint_command", qos);

      double dt = 0.01;
      control_timer_ = create_wall_timer(std::chrono::duration<double>(dt),std::bind(&EngineAIRos2NodeTemplate::ControlCallback, this));

      return true;
    }

    Eigen::VectorXd q_real_;
    Eigen::VectorXd qd_real_;
    Eigen::Vector3d w_real_;
    Eigen::Quaterniond quat_real_;

    Eigen::VectorXd q_des_;
    Eigen::VectorXd joint_kp_;
    Eigen::VectorXd joint_kd_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    interface_protocol::msg::JointCommand::SharedPtr joint_command_;

    // Subscribers
    rclcpp::Subscription<interface_protocol::msg::GamepadKeys>::SharedPtr gamepad_sub_;
    rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;
    rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<interface_protocol::msg::MotionState>::SharedPtr motion_state_sub_;
    // Publisher
    rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_pub_;

    // Message storage
    interface_protocol::msg::GamepadKeys::SharedPtr latest_gamepad_;
    interface_protocol::msg::ImuInfo::SharedPtr latest_imu_;
    interface_protocol::msg::JointState::SharedPtr latest_joint_state_;
    interface_protocol::msg::MotionState::SharedPtr latest_motion_state_;
  };
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<engineai_ros2_node_template::EngineAIRos2NodeTemplate> node = std::make_shared<engineai_ros2_node_template::EngineAIRos2NodeTemplate>();
  node->Initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
