#Create library "motion" using MotionPlanner.cpp
add_library(motion
    src/MotionPlanner.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(motion
    PUBLIC
        include
)

target_link_libraries(motion
    PUBLIC
        navigation
)