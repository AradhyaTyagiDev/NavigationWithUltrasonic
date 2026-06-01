#!/usr/bin/env sh
set -eu

BUILD_DIR="/tmp/navigation_robot_unit_tests"
BINARY="$BUILD_DIR/unit_tests"

mkdir -p "$BUILD_DIR"

g++ -std=c++17 \
    -Itest/unit \
    -Ilib/shared \
    -Ilib/shared/core \
    -Ilib/shared/interfaces \
    test/unit/main.cpp \
    test/unit/modules/TestUltrasonicFilter.cpp \
    test/unit/modules/TestObstacleManager.cpp \
    test/unit/modules/TestNavigationManager.cpp \
    test/unit/modules/TestMotionPlanner.cpp \
    test/unit/modules/TestMotorController.cpp \
    test/unit/modules/TestRobotController.cpp \
    lib/shared/core/filter/src/UltrasonicFilter.cpp \
    lib/shared/core/obstacle/src/ObstacleManager.cpp \
    lib/shared/core/navigation/src/NavigationManager.cpp \
    lib/shared/core/motion/src/MotionPlanner.cpp \
    lib/shared/core/motor/controller/src/MotorController.cpp \
    lib/shared/core/robot/src/RobotController.cpp \
    -o "$BINARY"

"$BINARY"
