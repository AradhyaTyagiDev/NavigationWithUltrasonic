#Create library "sensor" using UltrasonicSensor.cpp
add_library(interfaces INTERFACE)

#Also allow compiler to search headers inside include folder.
target_include_directories(interfaces
    PUBLIC
        include
)