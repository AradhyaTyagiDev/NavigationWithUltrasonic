#!/usr/bin/env sh
set -eu

BUILD_DIR="/tmp/navigation_robot_core_coverage"
BINARY="$BUILD_DIR/core_coverage_tests"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

SOURCES="
lib/shared/core/filter/src/UltrasonicFilter.cpp
lib/shared/core/obstacle/src/ObstacleManager.cpp
lib/shared/core/navigation/src/NavigationManager.cpp
lib/shared/core/motion/src/MotionPlanner.cpp
lib/shared/core/motor/controller/src/MotorController.cpp
lib/shared/core/robot/src/RobotController.cpp
"

g++ -std=c++17 \
    -O0 \
    --coverage \
    -Itest \
    -Itest/unit \
    -Itest/integration \
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
    test/integration/scenarios/TestRobotPipelineScenarios.cpp \
    $SOURCES \
    -o "$BINARY"

"$BINARY"

echo
echo "Coverage report:"

for source in $SOURCES
do
    object_name="$(basename "$source" .cpp)"
    gcov -o "$BUILD_DIR/core_coverage_tests-$object_name.gcno" "$source" |
        grep -E "File '$source'|Lines executed" || true
done

echo
echo "Raw .gcov files are in the current directory if your gcov version emits them there."
echo "Target: use this as a trend tool toward 95% line coverage for shared/core."
