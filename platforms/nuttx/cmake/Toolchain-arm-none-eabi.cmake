# arm-none-eabi-gcc toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(TOOLCHAIN_TRIPLE arm-none-eabi)
set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_TRIPLE})
set(TOOLCHAIN_PREFIX ${TOOLCHAIN_TRIPLE}-)

if(MINGW OR CYGWIN OR WIN32)
	set(UTIL_SEARCH_CMD where)
elseif(UNIX OR APPLE)
	set(UTIL_SEARCH_CMD which)
endif()

execute_process(
	COMMAND ${UTIL_SEARCH_CMD} ${TOOLCHAIN_PREFIX}gcc
	OUTPUT_VARIABLE BINUTILS_PATH
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(ARM_TOOLCHAIN_DIR ${BINUTILS_PATH} DIRECTORY)

set(CMAKE_C_FLAGS_INIT " -B${ARM_TOOLCHAIN_DIR} -target arm-none-eabi -Wno-error -DCONFIG_WCHAR_BUILTIN ")
set(CMAKE_CXX_FLAGS_INIT " -B${ARM_TOOLCHAIN_DIR} -target arm-none-eabi -Wno-error -DCONFIG_WCHAR_BUILTIN ")
# Without that flag CMake is not able to pass test compilation check
if (${CMAKE_VERSION} VERSION_EQUAL "3.6.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.6")
	set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
else()
	set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostdlib")
endif()

# C
set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_TRIPLE})
if(NOT CMAKE_C_COMPILER)
	set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
endif()
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
	set(CMAKE_C_FLAGS "-target arm-none-eabi -Wno-error -DCONFIG_WCHAR_BUILTIN --sysroot=/opt/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/" CACHE STRING "" FORCE)
endif()

# CXX
set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_TRIPLE})
if(NOT CMAKE_CXX_COMPILER)
	set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "-target arm-none-eabi -Wno-error -DCONFIG_WCHAR_BUILTIN --sysroot=/opt/gcc-arm-none-eabi-7-2017-q4-major/arm-none-eabi/" CACHE STRING "" FORCE)
endif()

set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

# needed for test compilation
#set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# compiler tools
foreach(tool nm ld ranlib strip)
	string(TOUPPER ${tool} TOOL)
	find_program(CMAKE_${TOOL} ${TOOLCHAIN_PREFIX}${tool})
	if(CMAKE-${TOOL} MATCHES "NOTFOUND")
		message(FATAL_ERROR "could not find ${TOOLCHAIN_PREFIX}${tool}")
	endif()
endforeach()

set(CMAKE_OBJCOPY llvm-objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL llvm-size CACHE INTERNAL "size tool")

set(CMAKE_SYSROOT ${ARM_TOOLCHAIN_DIR}/../${TOOLCHAIN_TRIPLE})
set(CMAKE_FIND_ROOT_PATH ${ARM_TOOLCHAIN_DIR}/../${TOOLCHAIN_TRIPLE})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# os tools
foreach(tool grep make)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

# optional compiler tools
foreach(tool gdb gdbtui)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-none-eabi-${tool})
endforeach()
