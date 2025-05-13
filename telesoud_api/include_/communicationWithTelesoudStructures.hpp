#ifndef COMMUNICATION_WITH_TELESOUD_STRUCTURES_HPP
#define COMMUNICATION_WITH_TELESOUD_STRUCTURES_HPP

#include "InstructionToRobotCode.hpp"
#include <array>
#include <iostream>
#include <rpc/server.h>

constexpr const uint8_t NUMBER_OF_AXIS = 6;

struct InstructionFromTelesoudToRobot {
  uint16_t instructionCode = static_cast<uint16_t>(InstructionCode::STOP);
  std::array<double, NUMBER_OF_AXIS> pose1{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // xyzwpr or joints value
  std::array<double, NUMBER_OF_AXIS> pose2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // xyzwpr or joints value
  std::array<double, NUMBER_OF_AXIS> pose3{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    // xyzwpr or joints value
  std::array<double, NUMBER_OF_AXIS> speedVector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // end effector speed vector
  double speed = 0.0; // speed of goToPosition instruction
  bool freeDriveBtnStatus = false;
  MSGPACK_DEFINE_ARRAY(instructionCode, pose1, pose2, pose3, speedVector, speed, freeDriveBtnStatus)
};

struct InformationFromRobotToTelesoud {
  bool robotInFaultStatus = false;                                               // robot in fault status
  bool robotInSlaveModeStatus = false;                                           // robot in free drive
  std::array<double, NUMBER_OF_AXIS> robotPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // current robot xyzwpr position
  std::array<double, NUMBER_OF_AXIS> robotJoints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // current robot joints values
  std::string errorsAsString = "";                                               // Robot erros
  bool collisionStatus = false;
  bool emergencyStop = false;
  bool weldingTrigger_PlcSignal = false;
  int operatinMode = 1; // Auto = 0 | T4 = 1 | T2 = 2
  MSGPACK_DEFINE_ARRAY(robotInFaultStatus, robotInSlaveModeStatus, robotPose, robotJoints, errorsAsString,
                       collisionStatus, emergencyStop, weldingTrigger_PlcSignal, operatinMode)
};

#endif // COMMUNICATION_WITH_TELESOUD_STRUCTURES_HPP