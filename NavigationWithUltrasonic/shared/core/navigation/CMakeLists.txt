#Create library "navigation" using NavigationManager.cpp
add_library(navigation
    src/NavigationManager.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(navigation
    PUBLIC
        include
)

target_link_libraries(navigation
    PUBLIC
        obstacle
)