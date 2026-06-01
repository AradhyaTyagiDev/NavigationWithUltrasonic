#!/usr/bin/env sh
set -eu

BUILD_DIR="/tmp/navigation_robot_integration_tests"
BINARY="$BUILD_DIR/integration_tests"

mkdir -p "$BUILD_DIR"

g++ -std=c++17 \
    -Itest \
    -Itest/unit \
    -Itest/integration \
    -Ilib/shared \
    -Ilib/shared/core \
    -Ilib/shared/interfaces \
    test/integration/main.cpp \
    test/integration/scenarios/TestRobotPipelineScenarios.cpp \
    lib/shared/core/filter/src/UltrasonicFilter.cpp \
    lib/shared/core/obstacle/src/ObstacleManager.cpp \
    lib/shared/core/navigation/src/NavigationManager.cpp \
    lib/shared/core/motion/src/MotionPlanner.cpp \
    lib/shared/core/motor/controller/src/MotorController.cpp \
    lib/shared/core/robot/src/RobotController.cpp \
    -o "$BINARY"

"$BINARY"
