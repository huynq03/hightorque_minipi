# deploy/

This directory follows the layout of `unitree_rl_mjlab/deploy`. Code shared by every robot lives in `include/` and `src/`, in namespace `deploy`. Each robot directory, `robots/<robot>/`, holds only what is specific to that robot.

```
deploy/
├── include/                    shared headers
│   ├── isaaclab/               policy runtime: ManagerBasedRLEnv, observation/action managers, OrtRunner
│   ├── FSM/                    CtrlFSM, BaseState, FSMState, ControlContext, State_Passive/FixStand/RLBase
│   ├── common/                 Types, JointMapper, Safety, SystemReadiness
│   ├── policy/                 RLPolicyRunner, RobotArticulation
│   ├── input/                  InputManager (/joy, /cmd_vel), KeyboardReader
│   ├── hardware/               HardwareInterface, SimHardware + simulated UDP bus (LowLevelBus, LowLevelUdpLink)
│   └── app/                    DeployApp: the whole node body (config, FSM, policy, readiness, diagnostics)
├── src/                        shared sources, see below
├── tools/policy_check.cpp      checks one policy package; built by each robot package
├── cmake/deploy_sources.cmake  shared source list, included by each robot's CMakeLists
└── robots/mini_pi/             catkin package `mini_pi_fsm`, Mini-Pi only
    ├── include/RobotSpec.h     dof count and robot name, read by the shared code
    ├── include/, src/          HardwareBackend (factory), HighTorqueHardware, VendorConflict
    ├── main.cpp                fills deploy::RobotApp and calls deploy::runDeployNode()
    ├── config/, launch/
    └── CMakeLists.txt, package.xml
```

## src/

`src/` uses the same subfolders as `include/`. All of these files are listed in `cmake/deploy_sources.cmake`.

```
src/
├── isaaclab/
│   ├── algorithms/algorithms.cpp           ONNX Runtime inference (OrtRunner); needs ISAACLAB_WITH_ONNXRUNTIME
│   ├── envs/manager_based_rl_env.cpp       builds the env from deploy.yaml and steps obs -> model -> action
│   └── manager/observation_manager.cpp     observation terms, history, scaling and clipping
├── FSM/
│   ├── CtrlFSM.cpp                         1 kHz control thread, state transitions, RL-entry guard
│   ├── FSMState.cpp                        transitions from fsm.yaml; post_run() hands off to Safety::publish
│   ├── State_Passive.cpp                   damping hold (kp = 0)
│   ├── State_FixStand.cpp                  interpolates from the measured pose to the stand pose
│   └── State_RLBase.cpp                    runs the policy thread and applies its latest command
├── common/
│   ├── JointMapper.cpp                     motor <-> robot order, sign and offset (mapping.yaml)
│   ├── Safety.cpp                          runtime checks, fault latch, clamping, the single exit to hardware
│   └── SystemReadiness.cpp                 startup gate: waits for motor and IMU feedback before the FSM starts
├── policy/
│   └── RLPolicyRunner.cpp                  resolves the policy package; 50 Hz policy thread; latest()
├── input/
│   ├── InputManager.cpp                    /joy, /cmd_vel and keyboard -> FSM requests + velocity command
│   └── KeyboardReader.cpp                  raw terminal key reader thread
├── hardware/
│   └── SimHardware.cpp                     `sim` backend: LowCmd/LowState over UDP to the MuJoCo bridge
└── app/
    └── DeployApp.cpp                       runDeployNode(): config, backend, policy, input, readiness, FSM,
                                            diagnostics, shutdown; builds the shared `sim` backend
```

Header-only: `FSM/BaseState.h` (state registry, `REGISTER_FSM`), `FSM/ControlContext.h`, `common/Types.h`, `policy/RobotArticulation.h`, `hardware/HardwareInterface.h`, `hardware/SimLowLevelClient.h`, `hardware/LowLevelBus.h` and `hardware/LowLevelUdpLink.h`.

## How the shared code is bound to a robot

- Shared headers `#include "RobotSpec.h"` by bare name. Each robot's CMakeLists puts its own `include/` on the include path, so the same source compiles for whichever robot is being built. Unitree does the same with each robot's `Types.h`.
- `RobotSpec.h` provides `robot_spec::kNumDof`. `deploy::NUM_DOF` is derived from it, and it sets the size of every `JointArray` and the width of the simulated bus.
- The robot's `main.cpp` provides a `RobotApp`: the ROS package name, the node name, the default backend, and `create_backend`, which builds the real backend. The shared code builds the `sim` backend itself, so every robot gets it.

## Real vs. sim

| | Real | Sim |
|---|:-:|:-:|
| All of `include/` + `src/` except `hardware/Sim*` and `LowLevel*` | ✓ | ✓ |
| `hardware/SimHardware`, `SimLowLevelClient`, `LowLevelBus`, `LowLevelUdpLink` | | ✓ |
| `robots/mini_pi/src/HighTorqueHardware.cpp` (compiled only with `MINI_PI_ENABLE_REAL_HW=ON`) | ✓ | |
| `robots/mini_pi/src/VendorConflict.cpp` (refuses the `hightorque` backend while a vendor process is running) | ✓ | |
| `robots/mini_pi/src/HardwareBackend.cpp` (real-backend factory; `sim` is built by `app/DeployApp`) | ✓ | |

## Adding a robot

1. `robots/<robot>/include/RobotSpec.h`.
2. A real backend that implements `deploy::HardwareInterface` (raw motor order, no mapping), plus a factory like `mini_pi::createHardwareBackend`.
3. A `main.cpp` (about 10 lines), plus a `CMakeLists.txt` and `package.xml` copied from `robots/mini_pi`. The shared sources come from `cmake/deploy_sources.cmake`; do not list them again.
4. `config/`: robot, mapping, fsm, safety and policy.yaml, plus the policy packages.
5. Optionally `simulate/robots/<robot>/`: include `hardware/LowLevelBus.h` with the robot's `include/` on the include path.

No file in `deploy/include` or `deploy/src` needs to change. See [doc/architecture.md](../doc/architecture.md) for details.
