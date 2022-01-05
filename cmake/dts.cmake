# SPDX-License-Identifier: Apache-2.0

file(MAKE_DIRECTORY ${PX4_BINARY_DIR}/dts/include/generated)

function(check_dtc_flag flag ok)
  execute_process(
    COMMAND
    ${DTC} ${flag} -v
    ERROR_QUIET
    OUTPUT_QUIET
    RESULT_VARIABLE dtc_check_ret
  )
  if (dtc_check_ret EQUAL 0)
    set(${ok} 1 PARENT_SCOPE)
  else()
    set(${ok} 0 PARENT_SCOPE)
  endif()
endfunction()

# Zephyr code can configure itself based on a KConfig'uration with the
# header file autoconf.h. There exists an analogous file devicetree_unfixed.h
# that allows configuration based on information encoded in DTS.
#
# Here we call on dtc, the gcc preprocessor and
# scripts/dts/gen_defines.py to generate various DT-related files at
# CMake configure-time.
#
# See the Devicetree user guide in the Zephyr documentation for details.
set(GEN_DEFINES_SCRIPT          ${PX4_SOURCE_DIR}/Tools/dts/gen_defines.py)
set(BOARD_DTS                   ${PX4_BINARY_DIR}/dts/board.dts)
# This contains the edtlib.EDT object created from zephyr.dts in Python's
# pickle data marshalling format (https://docs.python.org/3/library/pickle.html)
#
# Its existence is an implementation detail used to speed up further
# use of the devicetree by processes that run later on in the build,
# and should not be made part of the documentation.
set(EDT_PICKLE                  ${PX4_BINARY_DIR}/dts/edt.pickle)
set(DEVICETREE_UNFIXED_H        ${PX4_BINARY_DIR}/dts/include/generated/devicetree_unfixed.h)
set(DEVICE_EXTERN_H             ${PX4_BINARY_DIR}/dts/include/generated/device_extern.h)
set(DTS_POST_CPP                ${PX4_BINARY_DIR}/dts/px4.dts.pre)
set(DTS_DEPS                    ${PX4_BINARY_DIR}/dts/px4.dts.d)
# The location of a list of known vendor prefixes.
# This is relative to each element of DTS_ROOT.
set(VENDOR_PREFIXES             dts/bindings/vendor-prefixes.txt)

# Devicetree in CMake.
set(DTS_CMAKE_SCRIPT            ${PX4_SOURCE_DIR}/Tools/dts/gen_dts_cmake.py)
set(DTS_CMAKE                   ${PX4_BINARY_DIR}/dts/dts.cmake)

set(DTS_SOURCE ${PX4_BOARD_DIR}/board.dts)

# 'DTS_ROOT' is a list of directories where a directory tree with DT
# files may be found. It always includes the application directory,
# the board directory, and ${ZEPHYR_BASE}.
list(APPEND DTS_ROOT
  ${PX4_BOARD_DIR}
  ${PX4_SOURCE_DIR}
)
list(REMOVE_DUPLICATES DTS_ROOT)

# TODO: What to do about non-posix platforms where NOT CONFIG_HAS_DTS (xtensa)?
# Drop support for NOT CONFIG_HAS_DTS perhaps?
if(EXISTS ${DTS_SOURCE})
  set(SUPPORTS_DTS 1)
  if(BOARD_REVISION AND EXISTS ${PX4_BOARD_DIR}/${BOARD}_${BOARD_REVISION_STRING}.overlay)
    list(APPEND DTS_SOURCE ${PX4_BOARD_DIR}/${BOARD}_${BOARD_REVISION_STRING}.overlay)
  endif()
else()
  set(SUPPORTS_DTS 0)
endif()

set(dts_files
  ${DTS_SOURCE}
  ${shield_dts_files}
  )

if(SUPPORTS_DTS)
  if(DTC_OVERLAY_FILE)
    # Convert from space-separated files into file list
    string(REPLACE " " ";" DTC_OVERLAY_FILE_RAW_LIST "${DTC_OVERLAY_FILE}")
    foreach(file ${DTC_OVERLAY_FILE_RAW_LIST})
      file(TO_CMAKE_PATH "${file}" cmake_path_file)
      list(APPEND DTC_OVERLAY_FILE_AS_LIST ${cmake_path_file})
    endforeach()
    list(APPEND
      dts_files
      ${DTC_OVERLAY_FILE_AS_LIST}
      )
  endif()

  set(i 0)
  unset(DTC_INCLUDE_FLAG_FOR_DTS)
  foreach(dts_file ${dts_files})
    list(APPEND DTC_INCLUDE_FLAG_FOR_DTS
         -include ${dts_file})

    if(i EQUAL 0)
      message(STATUS "Found BOARD.dts: ${dts_file}")
    else()
      message(STATUS "Found devicetree overlay: ${dts_file}")
    endif()

    math(EXPR i "${i}+1")
  endforeach()

  unset(DTS_ROOT_SYSTEM_INCLUDE_DIRS)
  unset(DTS_ROOT_BINDINGS)
  foreach(dts_root ${DTS_ROOT})
    foreach(dts_root_path
        src/include
        dts/common
        dts/${CONFIG_ARCH}
        dts
        platforms/nuttx/src/px4/stm/hal_stm32/dts
        )
      get_filename_component(full_path ${dts_root}/${dts_root_path} REALPATH)
      if(EXISTS ${full_path})
        list(APPEND
          DTS_ROOT_SYSTEM_INCLUDE_DIRS
          -isystem ${full_path}
          )
      endif()
    endforeach()

    set(bindings_path ${dts_root}/dts/bindings)
    if(EXISTS ${bindings_path})
      list(APPEND
        DTS_ROOT_BINDINGS
        ${bindings_path}
        )
    endif()

    set(vendor_prefixes ${dts_root}/${VENDOR_PREFIXES})
    if(EXISTS ${vendor_prefixes})
      list(APPEND EXTRA_GEN_DEFINES_ARGS --vendor-prefixes ${vendor_prefixes})
    endif()
  endforeach()

  # Cache the location of the root bindings so they can be used by
  # scripts which use the build directory.
  set(CACHED_DTS_ROOT_BINDINGS ${DTS_ROOT_BINDINGS} CACHE INTERNAL
    "DT bindings root directories")

  if(NOT DEFINED CMAKE_DTS_PREPROCESSOR)
    set(CMAKE_DTS_PREPROCESSOR ${CMAKE_C_COMPILER})
  endif()

  # TODO: Cut down on CMake configuration time by avoiding
  # regeneration of devicetree_unfixed.h on every configure. How
  # challenging is this? What are the dts dependencies? We run the
  # preprocessor, and it seems to be including all kinds of
  # directories with who-knows how many header files.

  # Run the preprocessor on the DTS input files. We are leaving
  # linemarker directives enabled on purpose. This tells dtlib where
  # each line actually came from, which improves error reporting.
  execute_process(
    COMMAND ${CMAKE_DTS_PREPROCESSOR}
    -x assembler-with-cpp
    -nostdinc
    ${DTS_ROOT_SYSTEM_INCLUDE_DIRS}
    ${DTC_INCLUDE_FLAG_FOR_DTS}  # include the DTS source and overlays
    ${NOSYSDEF_CFLAG}
    -D__DTS__
    ${DTS_EXTRA_CPPFLAGS}
    -E   # Stop after preprocessing
    -MD  # Generate a dependency file as a side-effect
    -MF ${DTS_DEPS}
    -o ${DTS_POST_CPP}
    ${PX4_SOURCE_DIR}/platforms/common/empty.c
    WORKING_DIRECTORY ${PX4_SOURCE_DIR}
    RESULT_VARIABLE ret
    )
  if(NOT "${ret}" STREQUAL "0")
    message(FATAL_ERROR "command failed with return code: ${ret}")
  endif()

  # Parse the generated dependency file to find the DT sources that
  # were included, including any transitive includes, and then add
  # them to the list of files that trigger a re-run of CMake.
  # toolchain_parse_make_rule(${DTS_DEPS}
  #   include_files # Output parameter
  #   )

  # set_property(DIRECTORY APPEND PROPERTY
  #   CMAKE_CONFIGURE_DEPENDS
  #   ${include_files}
  #   ${GEN_DEFINES_SCRIPT}
  #   ${DTS_CMAKE_SCRIPT}
  #   )

  #
  # Run gen_defines.py to create a header file, zephyr.dts, and edt.pickle.
  #

  string(REPLACE ";" " " EXTRA_DTC_FLAGS_RAW "${EXTRA_DTC_FLAGS}")
  set(CMD_EXTRACT ${PYTHON_EXECUTABLE} ${GEN_DEFINES_SCRIPT}
  --dts ${DTS_POST_CPP}
  --dtc-flags '${EXTRA_DTC_FLAGS_RAW}'
  --bindings-dirs ${DTS_ROOT_BINDINGS}
  --header-out ${DEVICETREE_UNFIXED_H}
  --device-header-out ${DEVICE_EXTERN_H}
  --dts-out ${BOARD_DTS} # for debugging and dtc
  --edt-pickle-out ${EDT_PICKLE}
  ${EXTRA_GEN_DEFINES_ARGS}
  )

  execute_process(
    COMMAND ${CMD_EXTRACT}
    WORKING_DIRECTORY ${PX4_BINARY_DIR}
    RESULT_VARIABLE ret
    )
  if(NOT "${ret}" STREQUAL "0")
    message(FATAL_ERROR "gen_defines.py failed with return code: ${ret}")
  else()
    message(STATUS "Generated zephyr.dts: ${BOARD_DTS}")
    message(STATUS "Generated devicetree_unfixed.h: ${DEVICETREE_UNFIXED_H}")
    message(STATUS "Generated device_extern.h: ${DEVICE_EXTERN_H}")
  endif()

  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} ${DTS_CMAKE_SCRIPT}
    --edt-pickle ${EDT_PICKLE}
    --cmake-out ${DTS_CMAKE}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    RESULT_VARIABLE ret
    )
  if(NOT "${ret}" STREQUAL "0")
    message(FATAL_ERROR "gen_dts_cmake.py failed with return code: ${ret}")
  else()
    message(STATUS "Including generated dts.cmake file: ${DTS_CMAKE}")
    include(${DTS_CMAKE})
  endif()

  #
  # Run dtc on the final devicetree source, just to catch any
  # warnings/errors from it. dtlib and edtlib parse the devicetree files
  # themselves, so we don't rely on dtc otherwise.
  #

  if(DTC)
  set(DTC_WARN_UNIT_ADDR_IF_ENABLED "")
  check_dtc_flag("-Wunique_unit_address_if_enabled" check)
  if (check)
    set(DTC_WARN_UNIT_ADDR_IF_ENABLED "-Wunique_unit_address_if_enabled")
  endif()
  set(DTC_NO_WARN_UNIT_ADDR "")
  check_dtc_flag("-Wno-unique_unit_address" check)
  if (check)
    set(DTC_NO_WARN_UNIT_ADDR "-Wno-unique_unit_address")
  endif()
  set(VALID_EXTRA_DTC_FLAGS "")
  foreach(extra_opt ${EXTRA_DTC_FLAGS})
    check_dtc_flag(${extra_opt} check)
    if (check)
      list(APPEND VALID_EXTRA_DTC_FLAGS ${extra_opt})
    endif()
  endforeach()
  set(EXTRA_DTC_FLAGS ${VALID_EXTRA_DTC_FLAGS})
  execute_process(
    COMMAND ${DTC}
    -O dts
    -o - # Write output to stdout, which we discard below
    -b 0
    -E unit_address_vs_reg
    ${DTC_NO_WARN_UNIT_ADDR}
    ${DTC_WARN_UNIT_ADDR_IF_ENABLED}
    ${EXTRA_DTC_FLAGS} # User settable
    ${BOARD_DTS}
    OUTPUT_QUIET # Discard stdout
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    RESULT_VARIABLE ret
    )
  if(NOT "${ret}" STREQUAL "0")
    message(FATAL_ERROR "command failed with return code: ${ret}")
  endif()
  endif(DTC)
else()
  file(WRITE ${DEVICETREE_UNFIXED_H} "/* WARNING. THIS FILE IS AUTO-GENERATED. DO NOT MODIFY! */")
  file(WRITE ${DEVICE_EXTERN_H} "/* WARNING. THIS FILE IS AUTO-GENERATED. DO NOT MODIFY! */")
endif(SUPPORTS_DTS)
