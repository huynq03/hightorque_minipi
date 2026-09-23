# CMake generated Testfile for 
# Source directory: /home/hightorque/huy/catkin_ws/src/mini_pi_fsm
# Build directory: /home/hightorque/huy/catkin_ws/build/mini_pi_fsm
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(joint_mapper_roundtrip "/home/hightorque/huy/catkin_ws/devel/lib/mini_pi_fsm/test_joint_mapper" "/home/hightorque/huy/catkin_ws/src/mini_pi_fsm/config/mapping.yaml")
set_tests_properties(joint_mapper_roundtrip PROPERTIES  _BACKTRACE_TRIPLES "/home/hightorque/huy/catkin_ws/src/mini_pi_fsm/CMakeLists.txt;84;add_test;/home/hightorque/huy/catkin_ws/src/mini_pi_fsm/CMakeLists.txt;0;")
add_test(safety_and_config "/home/hightorque/huy/catkin_ws/devel/lib/mini_pi_fsm/test_safety_and_config" "/home/hightorque/huy/catkin_ws/src/mini_pi_fsm/config")
set_tests_properties(safety_and_config PROPERTIES  _BACKTRACE_TRIPLES "/home/hightorque/huy/catkin_ws/src/mini_pi_fsm/CMakeLists.txt;86;add_test;/home/hightorque/huy/catkin_ws/src/mini_pi_fsm/CMakeLists.txt;0;")
