#ifndef MSGS_INSTRUCTION_TO_ROBOT_CODE_HPP_
#define MSGS_INSTRUCTION_TO_ROBOT_CODE_HPP_

#include <iostream>

// clang-format off
enum class InstructionCode : uint16_t {
  STOP                            =0, // stop all movements
  GET_ROBOT_DATA                  =1, // get fresh robot data (used to monitor position)
  SET_DYNAMIC_CARTESIAN_SPEED     =7, // TCP (end-effector or twist) speed vector
  START_DYNAMIC_CARTESIAN_MOVEMENT=8, // start the cartisian movement of the end-effector 
  PLAY_CARTESIAN_TRAJECTORY       =15,// start a cartesian trajectory
  PLAY_JOINT_TRAJECTORY           =16,// start a joint trajectory
  SET_TCP_COORDINATES             =19,
  ABORT_ALL_ERRORS                =21,// abort errors order
  ACTIVATE_CLEAR_SIGNAL           =22
};
// clang-format on
#endif // MSGS_INSTRUCTION_TO_ROBOT_CODE_HPP_