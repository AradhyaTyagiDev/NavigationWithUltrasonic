idf_component_register(
    SRCS
        "UltrasonicSensor.cpp"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        freertos
        esp_timer
)