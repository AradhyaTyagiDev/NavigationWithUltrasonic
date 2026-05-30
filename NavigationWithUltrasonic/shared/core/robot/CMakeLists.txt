#Create library "robot" using RobotController.cpp
add_library(robot
    src/RobotController.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(robot
    PUBLIC
        include
)

target_link_libraries(robot
    PUBLIC
        motion #it will add all of other modules
        interfaces
)