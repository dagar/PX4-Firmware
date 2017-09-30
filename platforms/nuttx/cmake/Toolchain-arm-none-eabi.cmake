# defines:
#
# NM
# OBJCOPY
# LD
# CXX_COMPILER
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# GENROMFS
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE

include(CMakeForceCompiler)

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER arm-none-eabi-gcc)
if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find arm-none-eabi-gcc compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER arm-none-eabi-g++)
if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find arm-none-eabi-g++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-none-eabi-${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

# optional compiler tools
foreach(tool gdb gdbtui)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-none-eabi-${tool})
	if(NOT ${TOOL})
		#message(STATUS "could not find ${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo patch grep rm mkdir nm genromfs cp touch make unzip)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

set(cpu_flags)
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m7")
	add_compile_options(-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m4")
	add_compile_options(-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "cortex-m3")
	add_compile_options(-mcpu=cortex-m3 -mthumb -march=armv7-m)
else ()
	message(FATAL_ERROR "Processor not recognised in toolchain file")
endif()


add_compile_options($<$<COMPILE_LANGUAGE:C>:-fno-common -ffunction-sections -fdata-sections>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-common -ffunction-sections -fdata-sections>)
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>)
link_libraries(-nodefaultlibs -nostdlib -Wl,--warn-common,--gc-sections)

# where is the target environment 
set(CMAKE_FIND_ROOT_PATH get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

#=============================================================================
# check required toolchain variables
#

if (NOT CMAKE_C_COMPILER_ID)
	message(FATAL_ERROR "Toolchain/ must define CMAKE_C_COMPILER_ID")
endif()

if (NOT CMAKE_CXX_COMPILER_ID)
	message(FATAL_ERROR "Toolchain must define CMAKE_CXX_COMPILER_ID")
endif()

# print full c compiler version
execute_process(COMMAND ${CMAKE_C_COMPILER} --version
		OUTPUT_VARIABLE c_compiler_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
STRING(REGEX MATCH "[^\n]*" c_compiler_version_short ${c_compiler_version})
message(STATUS "C compiler: ${c_compiler_version_short}")

# print full c++ compiler version
execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version
		OUTPUT_VARIABLE cxx_compiler_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
STRING(REGEX MATCH "[^\n]*" cxx_compiler_version_short ${cxx_compiler_version})
message(STATUS "C++ compiler: ${cxx_compiler_version_short}")
