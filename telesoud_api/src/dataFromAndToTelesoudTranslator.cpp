#include <../include/dataFromAndToTelesoudTranslator.hpp>

DataFromAndToTelesoudTranslator::DataFromAndToTelesoudTranslator(rclcpp::Node::SharedPtr node) : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "THE TELESOUD TRANSLATOR INSTANCIED");
}

void DataFromAndToTelesoudTranslator::displayReceivedDataFromTelesoud(
    const InstructionFromTelesoudToRobot &instruction) const {
  RCLCPP_INFO(node_->get_logger(), "instruction code = %d, speed = %f", instruction.instructionCode, instruction.speed);
  RCLCPP_INFO(node_->get_logger(), "Pose1 = [%.3f, %.3f, %.3f, %.3f, %.3f,%.3f]", instruction.pose1[0],
              instruction.pose1[1], instruction.pose1[2], instruction.pose1[3], instruction.pose1[4],
              instruction.pose1[5]);
  RCLCPP_INFO(node_->get_logger(), "Pose2 = [%.3f, %.3f, %.3f, %.3f, %.3f,%.3f]", instruction.pose2[0],
              instruction.pose2[1], instruction.pose2[2], instruction.pose2[3], instruction.pose2[4],
              instruction.pose2[5]);
  RCLCPP_INFO(node_->get_logger(), "Pose3 = [%.3f, %.3f, %.3f, %.3f, %.3f,%.3f]", instruction.pose3[0],
              instruction.pose3[1], instruction.pose3[2], instruction.pose3[3], instruction.pose3[4],
              instruction.pose3[5]);
  RCLCPP_INFO(node_->get_logger(), "speedVector = [%.3f, %.3f, %.3f, %.3f, %.3f,%.3f]", instruction.speedVector[0],
              instruction.speedVector[1], instruction.speedVector[2], instruction.speedVector[3],
              instruction.speedVector[4], instruction.speedVector[5]);
}

InformationFromRobotToTelesoud DataFromAndToTelesoudTranslator::getInformationToSendToTelesoud(
    const std::array<double, 6> &currenXyzwpr, const sensor_msgs::msg::JointState &currentJointStates,
    bool robotInFault, bool robotInFreeDrive, const std::string errorDescription, bool emergencyStop,
    bool weldingTrigger_PlcSignal, int operatinMode, bool collisionStatus) {

 //RCLCPP_INFO(node_->get_logger(), "DEBUG - currenXyzwpr avant envoi RPC:");
 //RCLCPP_INFO(node_->get_logger(), "X: %.6f, Y: %.6f, Z: %.6f, W: %.6f, P: %.6f, R: %.6f",
 //             currenXyzwpr[0], currenXyzwpr[1], currenXyzwpr[2], 
 //             currenXyzwpr[3], currenXyzwpr[4], currenXyzwpr[5]);

  InformationFromRobotToTelesoud infosFromRobot;
  infosFromRobot.robotInFaultStatus = robotInFault;
  infosFromRobot.robotInSlaveModeStatus = robotInFreeDrive;
  for (int i = 0; i < NUMBER_OF_AXIS; i++) {
    infosFromRobot.robotPose[i] = currenXyzwpr[i];  // TODO: UNCOMMENT THIS WHEN currenXyzwpr IS POPULATED WITH DATA
    infosFromRobot.robotJoints[i] = 1.0;//currentJointStates.position[i]; // TODO: UNCOMMENT THIS WHEN currentJointStates IS POPULATED WITH DATA
  }

// RCLCPP_INFO(node_->get_logger(), "DEBUG - robotPose après assignation:");
//  RCLCPP_INFO(node_->get_logger(), "X: %.6f, Y: %.6f, Z: %.6f, W: %.6f, P: %.6f, R: %.6f",
//              infosFromRobot.robotPose[0], infosFromRobot.robotPose[1], infosFromRobot.robotPose[2],
//              infosFromRobot.robotPose[3], infosFromRobot.robotPose[4], infosFromRobot.robotPose[5]);

  infosFromRobot.errorsAsString = errorDescription;
  infosFromRobot.emergencyStop = emergencyStop;
  infosFromRobot.weldingTrigger_PlcSignal = weldingTrigger_PlcSignal;
  infosFromRobot.operatinMode = operatinMode;
  infosFromRobot.collisionStatus = collisionStatus;
  return infosFromRobot;
}
