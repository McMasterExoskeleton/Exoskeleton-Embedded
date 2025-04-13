# CMake generated Testfile for 
# Source directory: /home/dylan-exo/motor_api/Exoskeleton-Embedded
# Build directory: /home/dylan-exo/motor_api/Exoskeleton-Embedded/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(MotorAPITest "/home/dylan-exo/motor_api/Exoskeleton-Embedded/build/motor_api_test")
set_tests_properties(MotorAPITest PROPERTIES  _BACKTRACE_TRIPLES "/home/dylan-exo/motor_api/Exoskeleton-Embedded/CMakeLists.txt;33;add_test;/home/dylan-exo/motor_api/Exoskeleton-Embedded/CMakeLists.txt;0;")
subdirs("googletest")
