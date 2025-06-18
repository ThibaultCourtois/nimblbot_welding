#include "interface_rviz_plugin/interface_rviz_plugin.hpp"
#include <std_msgs/msg/bool.hpp>

namespace interface_rviz_plugin
{
  TeleopGainPanel::TeleopGainPanel(QWidget* parent)
    : rviz_common::Panel(parent)
    , pos_gain_(1.0)
    , quat_gain_(1.0)
    , modular_gain_(1.0)
    , current_modular_velocity_(0.0)
    , current_robot_state_("pause")
    , modular_command_active_(false)
    , current_modular_mode_("AZIMUTH")
    , emergency_stop_active_(false)
    , current_tcp_x_(0.0)
    , current_tcp_y_(0.0)
    , current_tcp_z_(0.0)
    , current_tcp_roll_(0.0)
    , current_tcp_pitch_(0.0)
    , current_tcp_yaw_(0.0)
  {
    // Initialize ROS 2 node
    node_ = std::make_shared<rclcpp::Node>("teleop_gain_panel_node");
    
    // Create publishers
    pos_gain_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/rviz_plugin/pos_gain", 10);
    
    quat_gain_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/rviz_plugin/quat_gain", 10);

    modular_gain_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/rviz_plugin/modular_gain", 10);
    
    clear_tcp_trace_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/rviz_plugin/clear_tcp_trace", 10);

    modular_command_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/rviz_plugin/modular_command_state", 10);
    
    emergency_stop_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/rviz_plugin/emergency_stop", 10);
    
    // Create subsribers 
    robot_state_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "/welding_command_handler/robotState", 
        10, 
        std::bind(&TeleopGainPanel::robotStateCallback, this, std::placeholders::_1)
        );
    
    modular_velocity_indicator_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/welding_modular_control/modular_velocity_indicator", 
        10,
        std::bind(&TeleopGainPanel::modularVelocityIndicatorCallback, this, std::placeholders::_1)
        );
    
    modular_mode_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "/welding_modular_control/modular_mode", 
        10, 
        std::bind(&TeleopGainPanel::modularModeCallback, this, std::placeholders::_1)
        );
 
    tcp_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        "/welding_command_handler/tcp_pose", 
        10, 
        std::bind(&TeleopGainPanel::tcpPoseCallback, this, std::placeholders::_1)
        );

    //Create clients
    set_zeros_client_ = node_->create_client<std_srvs::srv::Empty>("/welding_cartesian_command/set_zeros_request");

    // Setup UI components
    setupUi();
    
    // Timer for ROS spin
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &TeleopGainPanel::spinOnce);
    timer_->start(100);  // 10 Hz
    
    // Publish initial values
    publishPosGain();
    publishQuatGain();
    publishModularGain();
  }

  TeleopGainPanel::~TeleopGainPanel()
  {
    timer_->stop();
  }

  void TeleopGainPanel::setupUi()
  {
    // Main layout
    auto layout = new QVBoxLayout;
    setLayout(layout);
    
    // Robot state label
    robot_state_label_ = new QLabel("Robot State: IDLE");
    robot_state_label_->setAlignment(Qt::AlignCenter);
    robot_state_label_->setStyleSheet("font-weight: bold; font-size 16px;");
    
    // Sliders container 
    auto sliders_container = new QHBoxLayout;
    
    // Position Gain Slider 
    auto pos_layout = new QVBoxLayout;
    pos_label_ = new QLabel("Position Gain:");
    pos_label_->setAlignment(Qt::AlignCenter);
    pos_slider_ = new QSlider(Qt::Horizontal);  
    pos_slider_->setMinimum(1);
    pos_slider_->setMaximum(10);
    pos_slider_->setValue(static_cast<int>(pos_gain_));
    pos_value_label_ = new QLabel(QString::number(pos_gain_, 'f', 1));
    pos_value_label_->setAlignment(Qt::AlignCenter);
    
    pos_layout->addWidget(pos_label_);
    pos_layout->addWidget(pos_slider_);
    pos_layout->addWidget(pos_value_label_);
    
    // Quaternion Gain Slider 
    auto quat_layout = new QVBoxLayout;
    quat_label_ = new QLabel("Quaternion Gain:");
    quat_label_->setAlignment(Qt::AlignCenter);
    quat_slider_ = new QSlider(Qt::Horizontal); 
    quat_slider_->setMinimum(1);
    quat_slider_->setMaximum(10);
    quat_slider_->setValue(static_cast<int>(quat_gain_));
    quat_value_label_ = new QLabel(QString::number(quat_gain_, 'f', 1));
    quat_value_label_->setAlignment(Qt::AlignCenter);
    
    quat_layout->addWidget(quat_label_);
    quat_layout->addWidget(quat_slider_);
    quat_layout->addWidget(quat_value_label_);

    // TCP Pose Display
    auto tcp_group_box = new QGroupBox("TCP Pose");
    auto tcp_main_layout = new QHBoxLayout;
    tcp_group_box->setLayout(tcp_main_layout);

    // Position column
    auto tcp_pos_layout = new QVBoxLayout;
    auto tcp_pos_title = new QLabel("Position (mm)");
    tcp_pos_title->setAlignment(Qt::AlignCenter);
    tcp_pos_title->setStyleSheet("font-weight: bold;");

    tcp_pos_x_label_ = new QLabel("X: 0.0");
    tcp_pos_y_label_ = new QLabel("Y: 0.0");
    tcp_pos_z_label_ = new QLabel("Z: 0.0");

    tcp_pos_layout->addWidget(tcp_pos_title);
    tcp_pos_layout->addWidget(tcp_pos_x_label_);
    tcp_pos_layout->addWidget(tcp_pos_y_label_);
    tcp_pos_layout->addWidget(tcp_pos_z_label_);

    // Orientation column
    auto tcp_orient_layout = new QVBoxLayout;
    auto tcp_orient_title = new QLabel("Orientation (deg)");
    tcp_orient_title->setAlignment(Qt::AlignCenter);
    tcp_orient_title->setStyleSheet("font-weight: bold;");

    tcp_orient_w_label_ = new QLabel("W: 0.0");
    tcp_orient_p_label_ = new QLabel("P: 0.0");
    tcp_orient_r_label_ = new QLabel("R: 0.0");

    tcp_orient_layout->addWidget(tcp_orient_title);
    tcp_orient_layout->addWidget(tcp_orient_w_label_);
    tcp_orient_layout->addWidget(tcp_orient_p_label_);
    tcp_orient_layout->addWidget(tcp_orient_r_label_);

    tcp_main_layout->addLayout(tcp_pos_layout);
    tcp_main_layout->addLayout(tcp_orient_layout);

    // Modular Gain Slider
    auto modular_gain_layout = new QVBoxLayout;
    modular_gain_label_ = new QLabel("Modular Gain:");
    modular_gain_label_->setAlignment(Qt::AlignCenter);
    modular_gain_slider_ = new QSlider(Qt::Horizontal);
    modular_gain_slider_->setMinimum(1);   // 1.0
    modular_gain_slider_->setMaximum(10);  // 10.0
    modular_gain_slider_->setValue(static_cast<int>(modular_gain_));
    modular_gain_value_label_ = new QLabel(QString::number(modular_gain_, 'f', 1));
    modular_gain_value_label_->setAlignment(Qt::AlignCenter);
    
    modular_gain_layout->addWidget(modular_gain_label_);
    modular_gain_layout->addWidget(modular_gain_slider_);
    modular_gain_layout->addWidget(modular_gain_value_label_);
    
    sliders_container->addLayout(pos_layout);
    sliders_container->addLayout(quat_layout);
    sliders_container->addLayout(modular_gain_layout);

    // Modular command button 
    auto modular_command_layout = new QVBoxLayout;
    modular_command_button_ = new QPushButton("Modular Command");
    modular_command_button_->setCheckable(true);
    modular_command_button_->setChecked(false);
    connect(modular_command_button_, &QPushButton::clicked, this, &TeleopGainPanel::toggleModularCommand);
    
    // Modular mode
    modular_mode_label_ = new QLabel("Current mode: AZIMUTH");
    modular_mode_label_->setAlignment(Qt::AlignCenter);
    modular_mode_label_->setVisible(modular_command_active_); 
    
    // Modular velocity indicator 
    modular_velocity_indicator_label_ = new QLabel("Current speed: 0.00");
    modular_velocity_indicator_label_->setAlignment(Qt::AlignCenter);
    modular_velocity_indicator_label_->setVisible(modular_command_active_);
    
    modular_command_layout->addWidget(modular_command_button_);
    modular_command_layout->addWidget(modular_mode_label_);
    modular_command_layout->addWidget(modular_velocity_indicator_label_);
    
    // Clear TCP trace et Set Zeros boutons
    auto buttons_layout = new QHBoxLayout;
    clear_tcp_trace_button_ = new QPushButton("Clear TCP Trace");
    connect(clear_tcp_trace_button_, &QPushButton::clicked, this, &TeleopGainPanel::clearTcpTrace);
    
    set_zeros_button_ = new QPushButton("Set Zeros");
    connect(set_zeros_button_, &QPushButton::clicked, this, &TeleopGainPanel::setZeros);
    
    buttons_layout->addWidget(clear_tcp_trace_button_);
    buttons_layout->addWidget(set_zeros_button_);

    // Emergeny Stop Button 
    emergency_stop_button_ = new QPushButton("EMERGENCY STOP");
    emergency_stop_button_->setCheckable(true);
    emergency_stop_button_->setChecked(false);
    emergency_stop_button_->setMinimumHeight(60); 
    emergency_stop_button_->setStyleSheet("font-weight: bold; font-size: 14px;");
    connect(emergency_stop_button_, &QPushButton::clicked, this, &TeleopGainPanel::toggleEmergencyStop);

    auto emergency_layout = new QHBoxLayout;
    emergency_layout->addStretch();
    emergency_layout->addWidget(emergency_stop_button_);
    emergency_layout->addStretch();

    // Add all elements to the main layout
    layout->addWidget(robot_state_label_);
    layout->addLayout(buttons_layout);
    layout->addLayout(sliders_container);
    layout->addWidget(tcp_group_box);
    layout->addLayout(modular_command_layout);;
    layout->addLayout(emergency_layout);
    
    modular_mode_label_->setVisible(modular_command_active_);
    
    connect(pos_slider_, &QSlider::valueChanged, this, &TeleopGainPanel::updatePosGain);
    connect(quat_slider_, &QSlider::valueChanged, this, &TeleopGainPanel::updateQuatGain);
    connect(modular_gain_slider_, &QSlider::valueChanged, this, &TeleopGainPanel::updateModularGain);
  }

  void TeleopGainPanel::setZeros()
  {
    if (!set_zeros_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(node_->get_logger(), "Service /set_zeros not available");
      return;
    }
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    if (modular_command_active_) {
      modular_command_active_ = false;
      modular_command_button_->setChecked(false);
      modular_command_button_->setStyleSheet("");
      modular_mode_label_->setVisible(false);
      modular_velocity_indicator_label_->setVisible(false);
    }
    
    set_zeros_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        if (future.valid()) {
          RCLCPP_INFO(node_->get_logger(), "Set Zeros service called successfully");
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to call Set Zeros service");
        }
      }
    );
    
    RCLCPP_INFO(node_->get_logger(), "Set Zeros service call initiated");
  }

  void TeleopGainPanel::clearTcpTrace()
  {
    auto msg = std_msgs::msg::Empty();
    clear_tcp_trace_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Clear TCP trace message published");
  }

  void TeleopGainPanel::robotStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_robot_state_ = msg->data;
    updateRobotStateLabel();
    RCLCPP_DEBUG(node_->get_logger(), "Robot state update to: %s", current_robot_state_.c_str());
  }

  void TeleopGainPanel::updateRobotStateLabel()
  {
    QString styleSheet;

    if (current_robot_state_ == "IDLE") {
      styleSheet = "color: #F94C10; font-weight: bold;";
    } else if (current_robot_state_ == "DYNAMIC_MOVEMENT") {
      styleSheet = "color: #BC7AF9; font-weight: bold;";
    } else if (current_robot_state_ == "DYNAMIC_MOVEMENT_TELEOP_XYZ") {
      styleSheet = "color: #FFA1F5; font-weight: bold;";
    } else if (current_robot_state_ == "DYNAMIC_MOVEMENT_TRAJECTORY_EXECUTION") {
      styleSheet = "color: #687EFF; font-weight: bold;";
    } else if (current_robot_state_ == "CARTESIAN_TRAJECTORY") {
      styleSheet = "color: #80B3FF; font-weight: bold;";
    } else if (current_robot_state_ == "JOINT_TRAJECTORY") {
      styleSheet = "color: #98E4FF; font-weight: bold;";
    } else if (current_robot_state_ == "MODULAR_CONTROL") {
      styleSheet = "color: #4CCD99; font-weight: bold;";
    } else if (current_robot_state_ == "PAUSE") {
      styleSheet = "color: #C70039; font-weight: bold;";
    } else {
      styleSheet = "color: black; font-weight: bold;";
    }

    robot_state_label_->setText(QString("Robot State: %1").arg(QString::fromStdString(current_robot_state_)));
    robot_state_label_->setStyleSheet(styleSheet);
  }

  void TeleopGainPanel::updatePosGain(int value)
  {
    pos_gain_ = static_cast<double>(value);
    pos_value_label_->setText(QString::number(pos_gain_, 'f', 1));
    publishPosGain();
  }

  void TeleopGainPanel::updateQuatGain(int value)
  {
    quat_gain_ = static_cast<double>(value);
    quat_value_label_->setText(QString::number(quat_gain_, 'f', 1));
    publishQuatGain();
  }

  void TeleopGainPanel::publishPosGain()
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = pos_gain_;
    pos_gain_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Position gain set to: %f", pos_gain_);
  }

  void TeleopGainPanel::publishQuatGain()
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = quat_gain_;
    quat_gain_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Quaternion gain set to: %f", quat_gain_);
  }

  void TeleopGainPanel::spinOnce()
  {
    rclcpp::spin_some(node_);
  }

  void TeleopGainPanel::save(rviz_common::Config config) const
  {
    Panel::save(config);
    config.mapSetValue("pos_gain", pos_gain_);
    config.mapSetValue("quat_gain", quat_gain_);
    config.mapSetValue("modular_gain", modular_gain_); 
    config.mapSetValue("modular_command_active", modular_command_active_);
    config.mapSetValue("emergency_stop_active", emergency_stop_active_);
  }

  void TeleopGainPanel::load(const rviz_common::Config& config)
  {
    Panel::load(config);
    
    float pos_gain;
    if (config.mapGetFloat("pos_gain", &pos_gain))
    {
      pos_gain_ = pos_gain;
      pos_slider_->setValue(static_cast<int>(pos_gain_));
    }
    float quat_gain;
    if (config.mapGetFloat("quat_gain", &quat_gain))
    {
      quat_gain_ = quat_gain;
      quat_slider_->setValue(static_cast<int>(quat_gain_));
    }
    float modular_gain;
    if (config.mapGetFloat("modular_gain", &modular_gain))
    {
      modular_gain_ = modular_gain;
      modular_gain_slider_->setValue(static_cast<int>(modular_gain_));
    }
    bool modular_command_active;
    if (config.mapGetBool("modular_command_active", &modular_command_active))
    {
      modular_command_active_ = modular_command_active;
      modular_command_button_->setChecked(modular_command_active_);
      if (modular_command_active_) {
        modular_command_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
      }
    }
    bool emergency_stop_active;
    if (config.mapGetBool("emergency_stop_active", &emergency_stop_active))
    {
      emergency_stop_active_ = emergency_stop_active;
      emergency_stop_button_->setChecked(emergency_stop_active_);
      if (emergency_stop_active_) {
        emergency_stop_button_->setStyleSheet("background-color: #FF0000; color: white; font-weight: bold; font-size: 14px;");
      }
    }
  }

  void TeleopGainPanel::toggleModularCommand()
  {
    modular_command_active_ = modular_command_button_->isChecked();

    if (modular_command_active_) {
      modular_command_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
      modular_mode_label_->setVisible(true);
      modular_velocity_indicator_label_->setVisible(true);
    } else {
      modular_command_button_->setStyleSheet("");
      modular_mode_label_->setVisible(false);
      modular_velocity_indicator_label_->setVisible(false);
    }

    auto msg = std_msgs::msg::Bool();
    msg.data = modular_command_active_;
    modular_command_publisher_->publish(msg);

    RCLCPP_INFO(node_->get_logger(), "Modular command state changed to: %s",
                 modular_command_active_ ? "activated" : "deactivated");
  }

  void TeleopGainPanel::modularModeCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_modular_mode_ = msg->data;
    modular_mode_label_->setText(QString("Current mode: %1").arg(QString::fromStdString(current_modular_mode_)));
    RCLCPP_DEBUG(node_->get_logger(), "Modular mode updated to: %s", current_modular_mode_.c_str());
  }

  void TeleopGainPanel::modularVelocityIndicatorCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_modular_velocity_ = msg->data;
    modular_velocity_indicator_label_->setText(QString("Current speed: %1").arg(QString::number(current_modular_velocity_, 'f', 2)));
    RCLCPP_DEBUG(node_->get_logger(), "Modular velocity indicator updated: %f", current_modular_velocity_);
  }

  void TeleopGainPanel::updateModularGain(int value)
  {
    modular_gain_ = static_cast<double>(value); 
    modular_gain_value_label_->setText(QString::number(modular_gain_, 'f', 1));
    publishModularGain();
  }

  void TeleopGainPanel::publishModularGain()
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = modular_gain_;
    modular_gain_publisher_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Modular gain set to: %f", modular_gain_);
  }

  void TeleopGainPanel::toggleEmergencyStop()
  {
    emergency_stop_active_ = emergency_stop_button_->isChecked();

    if (emergency_stop_active_) {
      emergency_stop_button_->setStyleSheet("background-color: #FF0000; color: white; font-weight: bold; font-size: 14px;");
    } else {
      emergency_stop_button_->setStyleSheet("font-weight: bold; font-size: 14px;");
    }
    auto msg = std_msgs::msg::Bool();
    msg.data = emergency_stop_active_;
    emergency_stop_publisher_->publish(msg);

    RCLCPP_INFO(node_->get_logger(), "Emergency stop state changed to: %s",
                 emergency_stop_active_ ? "ACTIVATED" : "DEACTIVATED");
  }
  
  void TeleopGainPanel::tcpPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    current_tcp_x_ = msg->position.x * 1000.0; // Convert to mm
    current_tcp_y_ = msg->position.y * 1000.0;
    current_tcp_z_ = msg->position.z * 1000.0;
    
    // Convert quaternion to Euler using tf2
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, 
                     msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    
    m.getRPY(current_tcp_roll_, current_tcp_pitch_, current_tcp_yaw_);
    
    // Convert to degrees
    current_tcp_roll_ *= 180.0 / M_PI;
    current_tcp_pitch_ *= 180.0 / M_PI;
    current_tcp_yaw_ *= 180.0 / M_PI;
    
    // Update UI labels
    tcp_pos_x_label_->setText(QString("X: %1").arg(QString::number(current_tcp_x_, 'f', 1)));
    tcp_pos_y_label_->setText(QString("Y: %1").arg(QString::number(current_tcp_y_, 'f', 1)));
    tcp_pos_z_label_->setText(QString("Z: %1").arg(QString::number(current_tcp_z_, 'f', 1)));
    
    tcp_orient_w_label_->setText(QString("W: %1").arg(QString::number(current_tcp_yaw_, 'f', 1)));
    tcp_orient_p_label_->setText(QString("P: %1").arg(QString::number(current_tcp_pitch_, 'f', 1)));
    tcp_orient_r_label_->setText(QString("R: %1").arg(QString::number(current_tcp_roll_, 'f', 1)));
  }
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interface_rviz_plugin::TeleopGainPanel, rviz_common::Panel)
