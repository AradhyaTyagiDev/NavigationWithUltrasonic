# Level 2 Integration Tests

These tests run the real shared/core robot pipeline on desktop:

```text
FakeUltrasonicSensor
    -> UltrasonicFilter
    -> ObstacleManager
    -> NavigationManager
    -> MotionPlanner
    -> MotorController
    -> FakeMotorDriver
```

They do not use ESP-IDF, FreeRTOS, GPIO, PWM, RMT, threads, or real time.

Run from `firmware/esp32`:

```sh
./test/integration/run_integration_tests.sh
```

Run unit + integration tests with coverage instrumentation:

```sh
./test/run_core_coverage.sh
```

The scenarios cover:

- clear path forward motion
- approaching wall emergency behavior
- avoid-distance differential turning
- sensor dropout and recovery
- motor driver fault propagation
- automatic fault recovery
- motor busy / missed cycle recovery
- long clear-path soak
- flickering obstacle near avoid threshold
- emergency clear and resume
- controller timing violation supervision

Use these tests as Level 2 confidence. ESP32 task timing, RMT, GPIO, PWM, stack,
heap, watchdogs, and physical motor/sensor behavior belong to HIL and field
testing.
