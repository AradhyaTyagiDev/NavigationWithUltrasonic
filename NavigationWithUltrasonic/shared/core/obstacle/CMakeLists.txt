#Create library "obstacle" using ObstacleManager.cpp
add_library(obstacle
    src/ObstacleManager.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(obstacle
    PUBLIC
        include
)

target_link_libraries(obstacle
    PUBLIC
        filter
)