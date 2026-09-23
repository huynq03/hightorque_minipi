# 3.0.2 was the ROS Noetic boilerplate value, but CMake 4 removed compatibility
# with anything below 3.5 (and deprecates below 3.10) and refuses to configure. 3.10 is accepted by every
# CMake the robot or a PC is likely to have (Noetic on Ubuntu 20.04 ships
# 3.16), so raising the floor costs nothing and keeps both platforms building.
cmake_minimum_required(VERSION 3.10)
project(mini_pi_fsm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ============================ BUILD CONFIGURATION ===========================
# MINI_PI_ENABLE_REAL_HW controls exactly two things:
#   - whether src/hardware/HighTorqueHardware.cpp is compiled, and
#   - whether livelybot_serial is required as a catkin component.
# Nothing else in this package is conditional, and NO FSM behaviour depends on
# it. The FSM, JointMapper, SafetyManager, InputManager and PolicyRunner are
# compiled from the identical sources either way.
#
# ON  (default) -- the Mini-Pi itself. Both backends available.
# OFF           -- a PC. The HighTorque SDK ships only as an aarch64 build for
#                  the robot's own computer, so on x86_64 it cannot be linked
#                  at all; the simulation backend is used instead. See
#                  pixi.toml and simulate/README.md.
option(MINI_PI_ENABLE_REAL_HW
  "Build the real HighTorque backend (requires the aarch64 livelybot_serial SDK)"
  ON)

set(MINI_PI_CATKIN_COMPONENTS
  roscpp
  roslib          # ros::package::getPath() in src/main.cpp
  sensor_msgs
  geometry_msgs
)
if(MINI_PI_ENABLE_REAL_HW)
  list(APPEND MINI_PI_CATKIN_COMPONENTS livelybot_serial)
else()
  message(STATUS
    "mini_pi_fsm: MINI_PI_ENABLE_REAL_HW=OFF -- the 'hightorque' backend and "
    "livelybot_serial are excluded. This build provides the 'sim' backend only.")
endif()

find_package(catkin REQUIRED COMPONENTS ${MINI_PI_CATKIN_COMPONENTS})

find_package(yaml-cpp REQUIRED)

# ============================ INFERENCE BACKEND =============================
# MINI_PI_ENABLE_RL controls whether ONNX Runtime is linked and therefore
# whether rl::createOrtRunner() can produce a model. It gates NO other code:
# the whole rl/ layer, PolicyRunner and State_RLBase are compiled either way,
# and with it OFF the runner simply reports POLICY_NOT_CONFIGURED with
# "this build has no inference backend".
#
# WHERE ONNX RUNTIME COMES FROM, in order:
#   1. -DMINI_PI_ONNXRUNTIME_DIR=<dir>  a distribution directory containing
#      include/onnxruntime_cxx_api.h and lib/libonnxruntime.so. This is how the
#      ROBOT builds: the Mini-Pi image carries an aarch64 build at
#          /home/huynq/hightorque/onnxruntime-linux-aarch64-1.20.1
#   2. $CONDA_PREFIX -- how the PC builds, from the `onnxruntime-cpp` package
#      in pixi.toml.
#   3. the default CMake/system search path.
#
# Not finding it is a hard error with instructions, never a silent fallback to
# a build that cannot run a policy.
option(MINI_PI_ENABLE_RL "Link ONNX Runtime and enable RL policy execution" ON)

set(MINI_PI_ONNXRUNTIME_DIR "" CACHE PATH
  "Directory containing include/onnxruntime_cxx_api.h and lib/libonnxruntime.so")

if(MINI_PI_ENABLE_RL)
  set(_ort_hints "")
  if(MINI_PI_ONNXRUNTIME_DIR)
    list(APPEND _ort_hints ${MINI_PI_ONNXRUNTIME_DIR})
  elseif(DEFINED ENV{MINI_PI_ONNXRUNTIME_DIR})
    list(APPEND _ort_hints $ENV{MINI_PI_ONNXRUNTIME_DIR})
  endif()
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _ort_hints $ENV{CONDA_PREFIX})
  endif()

  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS ${_ort_hints}
    PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session)
  find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
    HINTS ${_ort_hints}
    PATH_SUFFIXES lib lib64)

  if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
      "mini_pi_fsm: ONNX Runtime not found.\n"
      "  Point the build at a distribution directory:\n"
      "      -DMINI_PI_ONNXRUNTIME_DIR=/path/to/onnxruntime-linux-<arch>-<ver>\n"
      "  (the Mini-Pi image has one at "
      "/home/huynq/hightorque/onnxruntime-linux-aarch64-1.20.1),\n"
      "  or install it into the active conda/pixi environment,\n"
      "  or build without policy execution:  -DMINI_PI_ENABLE_RL=OFF")
  endif()
  get_filename_component(ONNXRUNTIME_LIBDIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
  message(STATUS "mini_pi_fsm: ONNX Runtime ${ONNXRUNTIME_LIBRARY}")

  # Put the ONNX Runtime library directory AHEAD of the linker's built-in
  # search path, and record it as an rpath.
  #
  # Why this is needed, not cosmetic: the conda-forge onnxruntime build is
  # compiled against a newer libstdc++ than a typical system one. On this PC
  # it needs GLIBCXX_3.4.34 (std::__format::*), Debian 13's system libstdc++
  # provides up to 3.4.33, and the conda environment ships 3.4.36. Without
  # this, ld resolves libstdc++ from /usr/lib and the link fails with
  #     undefined reference to `std::__format::__with_encoding_conversion(
  #         std::locale const&)@GLIBCXX_3.4.34'
  # for EVERY executable, including the two that have nothing to do with RL.
  # Built-in search paths are consulted after all -L, so one -L is enough.
  #
  # On the robot this directory is the vendored ONNX Runtime's own lib/, which
  # contains no libstdc++, so the flags are inert there.
  link_directories(${ONNXRUNTIME_LIBDIR})
else()
  message(WARNING
    "mini_pi_fsm: MINI_PI_ENABLE_RL=OFF -- no inference backend. The RL layer "
    "still compiles, but PolicyRunner will always report POLICY_NOT_CONFIGURED "
    "and the RL state stays unreachable.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${MINI_PI_CATKIN_COMPONENTS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# livelybot_serial's INSTALLED package config (the prebuilt SDK on the Mini-Pi,
# /home/hightorque/install) exports only <prefix>/include, but its headers
# live in <prefix>/include/livelybot_serial/ and include each other as
# "crc16.h", "motor.h", "hardware/robot.h". The vendor's own sim2real
# CMakeLists works around it the same way. A SOURCE-space build of the SDK
# exports include/, include/crc, include/hardware directly, and the probes
# below then simply find nothing extra.
if(MINI_PI_ENABLE_REAL_HW)
  set(MINI_PI_LIVELYBOT_INCLUDE_DIRS "")
  foreach(_d ${livelybot_serial_INCLUDE_DIRS})
    foreach(_sub livelybot_serial livelybot_serial/crc livelybot_serial/hardware)
      if(EXISTS "${_d}/${_sub}")
        list(APPEND MINI_PI_LIVELYBOT_INCLUDE_DIRS "${_d}/${_sub}")
      endif()
    endforeach()
  endforeach()
  message(STATUS "mini_pi_fsm: livelybot_serial include dirs: "
                 "${livelybot_serial_INCLUDE_DIRS} ${MINI_PI_LIVELYBOT_INCLUDE_DIRS}")
  include_directories(${MINI_PI_LIVELYBOT_INCLUDE_DIRS})
endif()

# ---- the controller ---------------------------------------------------------
# One library, shared by every build. Splitting the FSM out from the backends
# into separate libraries was considered and rejected: the backend boundary is
# already a single source file plus one #if, and a three-library split would
# change the exported catkin target set and the install rules without removing
# a single line of duplication. There is no duplicated source either way.
set(MINI_PI_SOURCES
  src/control/JointMapper.cpp
  src/control/SafetyManager.cpp
  # Startup readiness (was State_Init) and the runtime safety gate
  # (was State_Fault). Neither is an FSM state any more.
  src/control/SystemReadiness.cpp
  src/control/SafetyGate.cpp
  src/input/InputManager.cpp
  src/input/KeyboardReader.cpp
  # RL deployment layer. Structural counterpart of Unitree's isaaclab/ tree;
  # see docs/rl_architecture.md. Compiled unconditionally -- only the inference
  # backend inside OrtRunner.cpp is conditional.
  src/rl/PolicyPackage.cpp
  src/rl/RobotView.cpp
  src/rl/ObservationManager.cpp
  src/rl/ActionManager.cpp
  src/rl/PolicyEnvironment.cpp
  src/rl/OrtRunner.cpp
  src/policy/PolicyRunner.cpp
  src/FSM/FSMState.cpp
  src/FSM/CtrlFSM.cpp
  src/FSM/State_Passive.cpp
  src/FSM/State_FixStand.cpp
  src/FSM/State_RLBase.cpp
  # Backend selection. The only translation unit that names a concrete
  # backend; see include/hardware/HardwareBackend.h.
  src/hardware/HardwareBackend.cpp
  # Simulation backend: pure sockets, buildable on every platform.
  src/hardware/SimHardware.cpp
  src/hardware/VendorConflict.cpp
)

# Real backend: the ONE source file that needs the HighTorque SDK.
if(MINI_PI_ENABLE_REAL_HW)
  list(APPEND MINI_PI_SOURCES src/hardware/HighTorqueHardware.cpp)
endif()

add_library(${PROJECT_NAME} ${MINI_PI_SOURCES})
if(MINI_PI_ENABLE_REAL_HW)
  target_compile_definitions(${PROJECT_NAME} PUBLIC MINI_PI_WITH_REAL_HW)
endif()
if(MINI_PI_ENABLE_RL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC MINI_PI_WITH_ONNXRUNTIME)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
endif()

# Warnings on for this package's own code. Third-party headers (ROS, yaml-cpp)
# arrive through ${catkin_INCLUDE_DIRS} and are not covered by these.
# Named once and reused by every target below, as simulate/CMakeLists.txt does,
# so a target added later cannot quietly be built without them.
set(MINI_PI_WARN -Wall -Wextra)
target_compile_options(${PROJECT_NAME} PRIVATE ${MINI_PI_WARN})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pthread
)
if(MINI_PI_ENABLE_RL)
  target_link_libraries(${PROJECT_NAME}
    ${ONNXRUNTIME_LIBRARY}
    -Wl,-rpath,${ONNXRUNTIME_LIBDIR})
endif()

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE ${MINI_PI_WARN})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# ---- non-hardware tests ----------------------------------------------------
# JointMapper round-trip: needs no ROS master, no hardware, no config file.
#   rosrun mini_pi_fsm test_joint_mapper <path/to/config/mapping.yaml>
add_executable(test_joint_mapper test/test_joint_mapper.cpp)
target_compile_options(test_joint_mapper PRIVATE ${MINI_PI_WARN})
add_dependencies(test_joint_mapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_joint_mapper ${PROJECT_NAME} ${catkin_LIBRARIES})

# SafetyManager behaviour, shipped-config assertions, the PolicyRunner refusal
# contract and the FixStand interpolator. Also needs no ROS master.
#   rosrun mini_pi_fsm test_safety_and_config <path/to/config>
add_executable(test_safety_and_config test/test_safety_and_config.cpp)
target_compile_options(test_safety_and_config PRIVATE ${MINI_PI_WARN})
add_dependencies(test_safety_and_config ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_safety_and_config ${PROJECT_NAME} ${catkin_LIBRARIES})

# Startup readiness and runtime safety, with a fake hardware backend: that a
# missing first packet WAITS instead of faulting, that the FSM starts in
# Passive, that a runtime fault latches and overrides the FSM command, and that
# recovery is explicit. Needs no ROS master and no hardware.
#   rosrun mini_pi_fsm test_startup_and_safety <path/to/config>
add_executable(test_startup_and_safety test/test_startup_and_safety.cpp)
target_compile_options(test_startup_and_safety PRIVATE ${MINI_PI_WARN})
add_dependencies(test_startup_and_safety ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_startup_and_safety ${PROJECT_NAME} ${catkin_LIBRARIES})

# Keyboard input: the key table in robot.yaml, keys -> FSM transition requests,
# keys -> VelocityCommand, and the source precedence. Needs no terminal, no ROS
# master and no hardware.
#   rosrun mini_pi_fsm test_input_keyboard <path/to/config>
add_executable(test_input_keyboard test/test_input_keyboard.cpp)
target_compile_options(test_input_keyboard PRIVATE ${MINI_PI_WARN})
add_dependencies(test_input_keyboard ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_input_keyboard ${PROJECT_NAME} ${catkin_LIBRARIES})

# The RL deployment layer: package resolution, the exact 47-D frame, the exact
# 15x47 history, action processing, policy<->robot joint order, ONNX Runtime
# and the full state->obs->model->action->RobotCommand path. Needs no ROS
# master and no hardware; it does need the generated test fixtures.
#   rosrun mini_pi_fsm test_rl_policy <path/to/config> <path/to/test/fixtures>
add_executable(test_rl_policy test/test_rl_policy.cpp)
target_compile_options(test_rl_policy PRIVATE ${MINI_PI_WARN})
add_dependencies(test_rl_policy ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_rl_policy ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_test(NAME joint_mapper_roundtrip
           COMMAND test_joint_mapper ${CMAKE_CURRENT_SOURCE_DIR}/config/mapping.yaml)
  add_test(NAME safety_and_config
           COMMAND test_safety_and_config ${CMAKE_CURRENT_SOURCE_DIR}/config)
  add_test(NAME startup_and_safety
           COMMAND test_startup_and_safety ${CMAKE_CURRENT_SOURCE_DIR}/config)
  add_test(NAME input_keyboard
           COMMAND test_input_keyboard ${CMAKE_CURRENT_SOURCE_DIR}/config)
  add_test(NAME rl_policy
           COMMAND test_rl_policy ${CMAKE_CURRENT_SOURCE_DIR}/config
                                  ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures)
endif()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  test_joint_mapper test_safety_and_config test_startup_and_safety
  test_input_keyboard test_rl_policy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
