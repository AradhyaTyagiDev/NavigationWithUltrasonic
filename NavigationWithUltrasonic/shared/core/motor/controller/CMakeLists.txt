#Create library "motor_controller" using MotorController.cpp
add_library(motor_controller
    src/MotorController.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(motor_controller
    PUBLIC
        include
)

target_link_libraries(motor_controller
    PUBLIC
        motion        
)