# Level 3 End-to-End Software Tests

These tests run the full platform-independent robot software graph for longer deterministic scenarios.

They still use fake sensor and motor boundaries, so they do not require ESP32 hardware, GPIO, PWM, RMT, Webots, or real time.

Run:

```sh
./test/e2e/run_e2e_tests.sh
```

This level validates:

- long-duration runtime health
- mission trace behavior across safe, caution, and avoid distances
- emergency dominance and explicit recovery
- sensor dropout and recovery
- bursty motor mutex contention
- motor fault recovery
- pipeline timing violation supervision
