#====================================================
# Motor Driver Module
#====================================================

idf_component_register(
    SRCS
        # Base HAL
        "IMotorDriver.cpp"
        "MotorDriverBase.cpp"
        # TB6612FNG Driver
        "TB6612FNG/TB6612Driver.cpp"

    INCLUDE_DIRS
        "."
        "TB6612FNG"

    REQUIRES
        driver
        freertos
        esp_timer
        log
)