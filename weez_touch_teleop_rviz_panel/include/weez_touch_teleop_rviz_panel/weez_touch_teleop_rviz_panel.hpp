#ifndef WEEZ_TOUCH_TELEOP_RVIZ_PANEL_HPP
#define WEEZ_TOUCH_TELEOP_RVIZ_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rviz_common/panel.hpp>

#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>

namespace weez_touch_teleop_rviz_panel
{
  class TeleopGainPanel : public rviz_common::Panel
  {
    Q_OBJECT

    public:
      // Constructor
      explicit TeleopGainPanel(QWidget* parent = nullptr);

      // Destructor
      ~TeleopGainPanel() override;

      // Override save/load configuration methods
      void save(rviz_common::Config config) const override;
      void load(const rviz_common::Config& config) override;

    private Q_SLOTS:
      void updatePosGain(int value);
      void updateQuatGain(int value);
      void updateModularGain(int value);
      void modularVelocityIndicatorCallback(const std_msgs::msg::Float64::SharedPtr msg);
      void clearTcpTrace();
      void spinOnce();
      void setZeros();
      void toggleModularCommand();
      void toggleEmergencyStop();
      void tcpPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    private:
      // Setup UI components
      void setupUi();

      // ROS publishers
      void publishPosGain();
      void publishQuatGain();
      void publishModularGain();
      
      // ROS subscribers
      void robotStateCallback(const std_msgs::msg::String::SharedPtr msg);
      void modularModeCallback(const std_msgs::msg::String::SharedPtr msg);
      void updateRobotStateLabel();

      // ROS clients
      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr set_zeros_client_;

      // ROS node, publishers and subscribers
      rclcpp::Node::SharedPtr node_;
      
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_gain_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr quat_gain_publisher_;
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_tcp_trace_publisher_; 
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr modular_command_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr modular_gain_publisher_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;

      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_subscriber_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr modular_mode_subscriber_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr modular_velocity_indicator_subscriber_;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tcp_pose_subscriber_;

      // UI components
      QSlider* pos_slider_;
      QSlider* quat_slider_;
      QSlider* modular_gain_slider_;
      QLabel* pos_value_label_;
      QLabel* pos_label_;
      QLabel* quat_value_label_;
      QLabel* quat_label_;
      QLabel* robot_state_label_;
      QLabel* modular_mode_label_;
      QLabel* cartesian_mode_label_;
      QLabel* modular_gain_label_;
      QLabel* modular_gain_value_label_;
      QLabel* modular_velocity_indicator_label_;
      QLabel* tcp_pos_x_label_;
      QLabel* tcp_pos_y_label_;
      QLabel* tcp_pos_z_label_;
      QLabel* tcp_orient_w_label_;
      QLabel* tcp_orient_p_label_;
      QLabel* tcp_orient_r_label_;
      QPushButton* clear_tcp_trace_button_; 
      QPushButton* set_zeros_button_;
      QPushButton* modular_command_button_;
      QPushButton* emergency_stop_button_;
      
      bool modular_command_active_;
      bool emergency_stop_active_;

      double pos_gain_;
      double quat_gain_;
      double modular_gain_;
      double current_modular_velocity_;
      double current_tcp_x_, current_tcp_y_, current_tcp_z_;
      double current_tcp_roll_, current_tcp_pitch_, current_tcp_yaw_;

      // command mode
      std::string current_modular_mode_;

      // robotState
      std::string current_robot_state_;

      // Timer for ROS spinning
      QTimer* timer_;
  };
} // namespace teleop_gain_panel
#endif // TELEOP_GAIN_PANEL_HPP
