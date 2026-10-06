// Mini-Pi deployment node.
//
// Structural counterpart of unitree_rl_mjlab/deploy/robots/g1/main.cpp. Only
// what is Mini-Pi's own is here: its ROS package and its hardware backends.
// Everything else -- config, FSM, Safety, policy, input, readiness,
// diagnostics -- is the shared deploy node (deploy/src/app/DeployApp.cpp).
#include "HardwareBackend.h"

int main(int argc, char** argv)
{
    deploy::RobotApp app;
    app.ros_package     = "mini_pi_fsm";
    app.node_name       = "mini_pi_fsm_node";
    app.default_backend = "hightorque";
    app.create_backend  = mini_pi::createHardwareBackend;
    return deploy::runDeployNode(argc, argv, app);
}
