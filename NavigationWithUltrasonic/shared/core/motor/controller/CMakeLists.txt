#Create library "motor" using MotorController.cpp
add_library(motor
    src/MotorController.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(motor
    PUBLIC
        include
)

target_link_libraries(motor
    PUBLIC
        motion        
)