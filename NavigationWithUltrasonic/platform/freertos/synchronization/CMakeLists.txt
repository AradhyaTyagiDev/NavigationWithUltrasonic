add_library(esp32_synchronization
    src/FreeRTOSMutex.cpp
)

target_include_directories(freertos_synchronization
    PUBLIC
        include
)

target_link_libraries(freertos_synchronization
    PUBLIC
        interfaces
    PRIVATE
        freertos
)