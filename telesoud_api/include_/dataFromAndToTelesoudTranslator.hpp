#ifndef DATA_FROM_AND_TO_TELESOUD_TRANSLATOR_HPP
#define DATA_FROM_AND_TO_TELESOUD_TRANSLATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <../include/communicationWithTelesoudStructures.hpp>

// Class that implement the api ros node
class DataFromAndToTelesoudTranslator {
public:
  DataFromAndToTelesoudTranslator(rclcpp::Node::SharedPtr node);

  void displayReceivedDataFromTelesoud(const InstructionFromTelesoudToRobot &instruction) const;
  InformationFromRobotToTelesoud
  getInformationToSendToTelesoud(const std::array<double, 6> &currenXyzwpr,
                                 const sensor_msgs::msg::JointState &currentJointStates, bool robotInFault,
                                 bool robotInFreeDrive, const std::string errorDescription, bool emergencyStop,
                                 bool weldingTrigger_PlcSignal, int operatinMode, bool collisionStatus);

private:
  std::shared_ptr<rclcpp::Node> node_;
};
#endif // DATA_FROM_AND_TO_TELESOUD_TRANSLATOR_HPP
