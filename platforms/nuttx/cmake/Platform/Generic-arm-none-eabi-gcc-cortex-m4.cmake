
set(cpu_flags "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${cpu_flags} ${CMAKE_C_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags} ${CMAKE_CXX_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__ ${CMAKE_ASM_FLAGS}" CACHE STRING "" FORCE)
