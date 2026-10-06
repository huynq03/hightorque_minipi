# Shared, robot-independent deploy sources (deploy/src). A robot's CMakeLists
# includes this file and adds ${DEPLOY_SOURCES} to its own library, so a new
# shared .cpp is listed once, here, for every robot.
#
# Sets:
#   DEPLOY_INCLUDE_DIR  deploy/include (the robot adds its own include/ too,
#                       for RobotSpec.h)
#   DEPLOY_SOURCES      every shared translation unit (tools excluded)
#   DEPLOY_POLICY_CHECK_SOURCE  deploy/tools/policy_check.cpp
get_filename_component(_deploy_dir "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(DEPLOY_INCLUDE_DIR "${_deploy_dir}/include")
set(DEPLOY_POLICY_CHECK_SOURCE "${_deploy_dir}/tools/policy_check.cpp")
set(DEPLOY_SOURCES
  # Policy runtime. Only the ONNX Runtime backend inside algorithms.cpp is
  # conditional (ISAACLAB_WITH_ONNXRUNTIME).
  ${_deploy_dir}/src/isaaclab/algorithms/algorithms.cpp
  ${_deploy_dir}/src/isaaclab/manager/observation_manager.cpp
  ${_deploy_dir}/src/isaaclab/envs/manager_based_rl_env.cpp
  ${_deploy_dir}/src/common/JointMapper.cpp
  # Every runtime safety check, the fault latch and the one exit to hardware.
  ${_deploy_dir}/src/common/Safety.cpp
  # Startup readiness: runs before CtrlFSM starts, not a state.
  ${_deploy_dir}/src/common/SystemReadiness.cpp
  ${_deploy_dir}/src/input/InputManager.cpp
  ${_deploy_dir}/src/input/KeyboardReader.cpp
  # The one policy thread the FSM uses.
  ${_deploy_dir}/src/policy/RLPolicyRunner.cpp
  ${_deploy_dir}/src/FSM/FSMState.cpp
  ${_deploy_dir}/src/FSM/CtrlFSM.cpp
  ${_deploy_dir}/src/FSM/State_Passive.cpp
  ${_deploy_dir}/src/FSM/State_FixStand.cpp
  ${_deploy_dir}/src/FSM/State_RLBase.cpp
  # Simulation backend: pure sockets, buildable on every platform.
  ${_deploy_dir}/src/hardware/SimHardware.cpp
  # The node body every robot's main.cpp calls.
  ${_deploy_dir}/src/app/DeployApp.cpp
)
