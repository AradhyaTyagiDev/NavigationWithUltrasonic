#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/.test_build/platform_host"
mkdir -p "$BUILD_DIR"

CXX="${CXX:-g++}"

"$CXX" -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-const-variable -O0 -g \
    -I"$ROOT_DIR/test/platform" \
    -I"$ROOT_DIR/test/platform/fake_espidf" \
    -I"$ROOT_DIR/test/unit" \
    -I"$ROOT_DIR/test/integration" \
    -I"$ROOT_DIR/../../shared" \
    -I"$ROOT_DIR/../../shared/core" \
    -I"$ROOT_DIR/../../platform/esp32" \
    "$ROOT_DIR/test/platform/fake_espidf/FakeEspIdf.cpp" \
    "$ROOT_DIR/../../platform/esp32/motor/TB6612FNG/src/TB6612Driver.cpp" \
    "$ROOT_DIR/../../platform/esp32/sensor/ultrasonic/src/UltrasonicSensor.cpp" \
    "$ROOT_DIR/../../platform/esp32/runtime/src/Esp32RobotRuntime.cpp" \
    "$ROOT_DIR/../../shared/core/filter/src/UltrasonicFilter.cpp" \
    "$ROOT_DIR/../../shared/core/obstacle/src/ObstacleManager.cpp" \
    "$ROOT_DIR/../../shared/core/navigation/src/NavigationManager.cpp" \
    "$ROOT_DIR/../../shared/core/motion/src/MotionPlanner.cpp" \
    "$ROOT_DIR/../../shared/core/motor/controller/src/MotorController.cpp" \
    "$ROOT_DIR/../../shared/core/robot/src/RobotController.cpp" \
    "$ROOT_DIR/test/platform/adapter/TestTB6612DriverHost.cpp" \
    "$ROOT_DIR/test/platform/adapter/TestUltrasonicSensorHost.cpp" \
    "$ROOT_DIR/test/platform/adapter/TestEsp32RobotRuntimeHost.cpp" \
    "$ROOT_DIR/test/platform/adapter/main.cpp" \
    -o "$BUILD_DIR/platform_host_adapter_tests"

"$BUILD_DIR/platform_host_adapter_tests"
