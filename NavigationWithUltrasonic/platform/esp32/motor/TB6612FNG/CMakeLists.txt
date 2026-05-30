#Create library "driver_tb6612" using TB6612Driver.cpp
add_library(motor
    src/TB6612Driver.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(motor
    PUBLIC
        include
)

target_link_libraries(motor
    PUBLIC
        freertos
        esp_timer
)