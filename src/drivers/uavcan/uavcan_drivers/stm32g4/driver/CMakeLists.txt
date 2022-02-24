include_directories(
    ./include
    )

add_library(uavcan_stm32g4_driver STATIC
    ./src/uc_stm32g4_can.cpp
    ./src/uc_stm32g4_clock.cpp
    ./src/uc_stm32g4_thread.cpp
    )

add_dependencies(uavcan_stm32g4_driver uavcan)

install(DIRECTORY include/uavcan_stm32g4 DESTINATION include)
install(TARGETS uavcan_stm32g4_driver DESTINATION lib)

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :)
