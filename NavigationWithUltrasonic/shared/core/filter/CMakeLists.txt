#Create library "filter" using UltrasonicFilter.cpp
add_library(filter
    src/UltrasonicFilter.cpp
)

#Also allow compiler to search headers inside include folder.
target_include_directories(filter
    PUBLIC
        include
)