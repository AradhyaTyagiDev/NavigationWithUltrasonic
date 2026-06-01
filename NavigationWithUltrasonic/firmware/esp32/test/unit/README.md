# Unit Test Harness

This directory contains desktop unit tests for the platform-independent robot
core. The tests intentionally avoid ESP-IDF, FreeRTOS, GPIO, PWM, Webots, and
real time.

Run from `firmware/esp32`:

```sh
./test/unit/run_unit_tests.sh
```

The harness uses:

- `framework/TestFramework.hpp`: tiny assertion and test registry layer.
- `fakes/Fakes.hpp`: fake timer, logger, mutex, ultrasonic sensor, and motor driver.
- `modules/`: module-level tests for the core pipeline.

The goal is fast feedback before integration, simulation, HIL, or real robot
testing.
