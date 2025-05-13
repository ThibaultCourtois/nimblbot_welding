#ifndef TELESOUD_APINODEE_HPP
#define TELESOUD_APINODEE_HPP

#include "rclcpp/rclcpp.hpp"
#include <communicationWithTelesoudStructures.hpp>
#include <dataFromAndToTelesoudTranslator.hpp>
#include <rpc/server.h>
#include <telesoud_msgs/msg/telesoud_instruction.hpp>
#include <telesoud_msgs/msg/robot_data.hpp>

// Class that implement the api ros node
class ApiNode : public rclcpp::Node {
public:
  explicit ApiNode();

private:

  bool robotInFault_ = false;
  bool robotInFreeDrive_ = false;
  std::string errorDescription_ = "";
  bool emergencyStop_ = false;
  bool weldingTrigger_PlcSignal_ = false;
  int operationMode_ = 1;
  bool collisionStatus_ = false;

  // Class initialization after construction
  rclcpp::TimerBase::SharedPtr initialize_timer;
  void initialize();

  // RPC Communication
  const uint16_t COMMUNICATION_PORT = 44490;

  // API Node
  std::shared_ptr<DataFromAndToTelesoudTranslator> telesoudTranslator_;
  rclcpp::CallbackGroup::SharedPtr api_callback_group_;
  sensor_msgs::msg::JointState current_joint_states_;
  geometry_msgs::msg::PoseStamped currentEndEffPose_;
  std::array<double, 6> currentEndEffXyzwpr_;
  
  //Instructions publisher to NimblBot welding framework
  rclcpp::Publisher<telesoud_msgs::msg::TelesoudInstruction>::SharedPtr instruction_publisher_;

  //Robot Data subscriber from NimblBot welding framework to Telesoud
  using RobotDataMsg = telesoud_msgs::msg::RobotData;
  rclcpp::Subscription<RobotDataMsg>::SharedPtr robot_data_subscription_;
  void robotDataCallback(const std::shared_ptr<RobotDataMsg> msg);
  
  // ============================================================
  //                  TELESOUD ORDERS MANAGEMENT
  // ============================================================
  // -------- Variables --------
  std::unique_ptr<rpc::server> telesoudCommunicationSrv_;
  int lastInstCodefronTelesoud = 100;

  // -------- Functions --------
  InformationFromRobotToTelesoud
  instructionsFromTelesoudManager(const InstructionFromTelesoudToRobot &instruction);
};

#endif // TELESOUD_APINODEE_HPP
