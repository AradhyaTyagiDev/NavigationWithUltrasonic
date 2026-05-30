#Create library "sensor" using UltrasonicSensor.cpp
add_library(sensor
    src/UltrasonicSensor.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(sensor
    PUBLIC
        include
)

target_link_libraries(sensor
    PUBLIC
        driver
        freertos
        esp_timer
)