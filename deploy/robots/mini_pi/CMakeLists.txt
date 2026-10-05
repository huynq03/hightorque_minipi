# 3.0.2 was the ROS Noetic boilerplate value, but CMake 4 removed compatibility
# with anything below 3.5 (and deprecates below 3.10) and refuses to configure. 3.10 is accepted by every
# CMake the robot or a PC is likely to have (Noetic on Ubuntu 20.04 ships
# 3.16), so raising the floor costs nothing and keeps both platforms building.
cmake_minimum_required(VERSION 3.10)
project(mini_pi_fsm)

# This package is deploy/robots/mini_pi. The robot-independent policy runtime
# (Unitree's isaaclab/ layout) lives two levels up in deploy/include/isaaclab
# and deploy/src/isaaclab and is compiled straight into this package -- no
# prebuilt library. The catkin workspace reaches this directory through a
# symlink, so resolve it first: a lexical "../.." from the link would land in
# the workspace instead of deploy/.
get_filename_component(MINI_PI_PKG_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(MINI_PI_DEPLOY_DIR "${MINI_PI_PKG_DIR}/../.." ABSOLUTE)
if(NOT EXISTS "${MINI_PI_DEPLOY_DIR}/include/isaaclab/envs/manager_based_rl_env.h")
  message(FATAL_ERROR "mini_pi_fsm: shared runtime not found at ${MINI_PI_DEPLOY_DIR}. "
                      "This package must stay at deploy/robots/mini_pi.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ============================ BUILD CONFIGURATION ===========================
# MINI_PI_ENABLE_REAL_HW controls exactly two things:
#   - whether src/HighTorqueHardware.cpp is compiled, and
#   - whether livelybot_serial is required as a catkin component.
# Nothing else in this package is conditional, and NO FSM behaviour depends on
# it. The FSM, JointMapper, Safety, InputManager and RLPolicyRunner are
# compiled from the identical sources either way.
#
# ON  (default) -- the Mini-Pi itself. Both backends available.
# OFF           -- a PC. The HighTorque SDK ships only as an aarch64 build for
#                  the robot's own computer, so on x86_64 it cannot be linked
#                  at all; the simulation backend is used instead. See
#                  pixi.toml and doc/simulation.md.
option(MINI_PI_ENABLE_REAL_HW
  "Build the real HighTorque backend (requires the aarch64 livelybot_serial SDK)"
  ON)

set(MINI_PI_CATKIN_COMPONENTS
  roscpp
  roslib          # ros::package::getPath() in main.cpp
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
# whether createOrtRunner() can produce a model. It gates NO other code:
# the whole isaaclab/ runtime, RLPolicyRunner and State_RLBase are compiled either way,
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
    "still compiles, but RLPolicyRunner will always report POLICY_NOT_CONFIGURED "
    "and the RL state stays unreachable.")
endif()

catkin_package(
  INCLUDE_DIRS include ${MINI_PI_DEPLOY_DIR}/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${MINI_PI_CATKIN_COMPONENTS}
)

include_directories(
  include
  ${MINI_PI_DEPLOY_DIR}/include
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
  # Shared, robot-independent policy runtime (deploy/src/isaaclab). Compiled
  # unconditionally -- only the ONNX Runtime backend inside algorithms.cpp is
  # conditional.
  ${MINI_PI_DEPLOY_DIR}/src/isaaclab/algorithms/algorithms.cpp
  ${MINI_PI_DEPLOY_DIR}/src/isaaclab/manager/observation_manager.cpp
  ${MINI_PI_DEPLOY_DIR}/src/isaaclab/envs/manager_based_rl_env.cpp
  # Mini-Pi.
  src/JointMapper.cpp
  # Every runtime safety check, the fault latch and the one exit to hardware.
  src/Safety.cpp
  src/InputManager.cpp
  src/KeyboardReader.cpp
  # The one policy thread the FSM uses.
  src/RLPolicyRunner.cpp
  # Startup readiness: runs before CtrlFSM starts, not a state.
  src/SystemReadiness.cpp
  src/FSMState.cpp
  src/CtrlFSM.cpp
  src/State_Passive.cpp
  src/State_FixStand.cpp
  src/State_RLBase.cpp
  # Backend selection. The only translation unit that names a concrete
  # backend; see include/HardwareBackend.h.
  src/HardwareBackend.cpp
  # Simulation backend: pure sockets, buildable on every platform.
  src/SimHardware.cpp
  src/VendorConflict.cpp
)

# Real backend: the ONE source file that needs the HighTorque SDK.
if(MINI_PI_ENABLE_REAL_HW)
  list(APPEND MINI_PI_SOURCES src/HighTorqueHardware.cpp)
endif()

add_library(${PROJECT_NAME} ${MINI_PI_SOURCES})
if(MINI_PI_ENABLE_REAL_HW)
  target_compile_definitions(${PROJECT_NAME} PUBLIC MINI_PI_WITH_REAL_HW)
endif()
if(MINI_PI_ENABLE_RL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ISAACLAB_WITH_ONNXRUNTIME)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
endif()

# Warnings on for this package's own code. Third-party headers (ROS, yaml-cpp)
# arrive through ${catkin_INCLUDE_DIRS} and are not covered by these.
# Named once and reused by every target below, as the simulator's CMakeLists does,
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

add_executable(${PROJECT_NAME}_node main.cpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE ${MINI_PI_WARN})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# Resolves and fully loads one policy package with the controller's own
# RLPolicyRunner, exactly as the node would; scripts/sim.sh runs it first.
#   rosrun mini_pi_fsm policy_check <path/to/config> [version]
add_executable(policy_check tools/policy_check.cpp)
target_compile_options(policy_check PRIVATE ${MINI_PI_WARN})
add_dependencies(policy_check ${catkin_EXPORTED_TARGETS})
target_link_libraries(policy_check ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node policy_check
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ ${MINI_PI_DEPLOY_DIR}/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
