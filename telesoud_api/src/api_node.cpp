#include "rclcpp/rclcpp.hpp"
#include "../include/api_node.hpp"

// ============================================================
//  NODE CREATION
// ============================================================
ApiNode::ApiNode() : Node("ApiNode", rclcpp::NodeOptions().use_intra_process_comms(true)) {
  // use one-time timer to initiate
  initialize_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ApiNode::initialize, this));
}

void ApiNode::initialize() {
  initialize_timer->cancel();
  telesoudTranslator_ = std::make_shared<DataFromAndToTelesoudTranslator>(shared_from_this());
	
  // ----------- Instructions publisher to NimblBot welding framework --------------
  instruction_publisher_ = this->create_publisher<interface_custom_msgs::msg::TelesoudInstruction>(
  		  "/telesoud/instructions"
		  , 10
		  );

  // ----------- RobotInfo subscriber from NimblBot welding framework to Telesoud -----------
  robot_data_subscription_ = this->create_subscription<interface_custom_msgs::msg::RobotData>(
		  "/translator/robotData", 
		  10, 
		  std::bind(&ApiNode::robotDataCallback, this, std::placeholders::_1)
		  ); 
  

  // ------ Callback group ---------
  api_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions optionsSubscr;
  optionsSubscr.callback_group = api_callback_group_;

  telesoudCommunicationSrv_ = std::make_unique<rpc::server>(COMMUNICATION_PORT);
  telesoudCommunicationSrv_->bind("communicationTelesoud",
                                  [this](const InstructionFromTelesoudToRobot &instructions) {
                                    return instructionsFromTelesoudManager(instructions);
                                  });
  constexpr size_t thread_count = 1;
  telesoudCommunicationSrv_->async_run(thread_count); // non-blocking call, handlers execute on one of the
                                                      // workers
}

// ------------- Info from NimblBot to telesoud ---------------------------
void ApiNode::robotDataCallback(const std::shared_ptr<RobotDataMsg> msg) {

  for (int i=0; i < NUMBER_OF_AXIS; i++){
    currentEndEffXyzwpr_[i] = msg->xyzwpr[i];
  }

  robotInFault_ = msg->robot_in_fault_status;
  robotInFreeDrive_ = msg->robot_in_slave_mode_status;
  errorDescription_ = msg->error_message;
  emergencyStop_ = msg->emergency_stop;
  weldingTrigger_PlcSignal_ = msg->welding_trigger_plc_signal;
  operationMode_ = msg->operation_mode;
  collisionStatus_ = msg->collision_status;

  RCLCPP_DEBUG(this->get_logger(), "Received robot info: position=[%.3f, %.3f, %.3f]", 
		  msg->xyzwpr[0], msg->xyzwpr[1], msg->xyzwpr[2]);
}

// ============================================================
//  TELESOUD API
// ============================================================
InformationFromRobotToTelesoud
ApiNode::instructionsFromTelesoudManager(const InstructionFromTelesoudToRobot &instFromTelesoud) {
  if (instFromTelesoud.instructionCode != lastInstCodefronTelesoud) {
   telesoudTranslator_->displayReceivedDataFromTelesoud(instFromTelesoud);
   lastInstCodefronTelesoud = instFromTelesoud.instructionCode;
  }

  // ----- Publishing --------
  auto msg = interface_custom_msgs::msg::TelesoudInstruction();
  msg.instruction_code = instFromTelesoud.instructionCode;

 //  Pose1, 2, 3 and speed vector to msg

  for (int i = 0; i < NUMBER_OF_AXIS; i ++){
    msg.pose1[i] = instFromTelesoud.pose1[i];
    msg.speed_vector[i] = instFromTelesoud.speedVector[i];
  }

  msg.speed = instFromTelesoud.speed;
  msg.free_drive_btn_status = instFromTelesoud.freeDriveBtnStatus;

  instruction_publisher_->publish(msg);


  return telesoudTranslator_->getInformationToSendToTelesoud(
      currentEndEffXyzwpr_, current_joint_states_, false, false,
      "", false, false, 1, false);
}
