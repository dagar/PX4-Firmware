#=============================================================================
#
#	px4_generate_builtin_commands
#
#	This function generates the builtin_commands.c src for posix
#
#	Usage:
#		px4_generate_builtin_commands(
#			MODULE_LIST <in-list>
#			OUT <file>)
#
#	Input:
#		MODULE_LIST	: list of modules
#
#	Output:
#		OUT	: stem of generated apps.cpp/apps.h ("apps")
#
#	Example:
#		px4_generate_builtin_commands(
#			OUT <generated-src> MODULE_LIST px4_simple_app)
#
function(px4_generate_builtin_commands)
	px4_parse_function_args(
		NAME px4_generate_builtin_commands
		ONE_VALUE OUT
		MULTI_VALUE MODULE_LIST
		REQUIRED MODULE_LIST OUT
		ARGN ${ARGN})

	set(builtin_apps_string)
	set(builtin_apps_decl_string)
	set(command_count 0)

	list(SORT MODULE_LIST)

	#message(STATUS "MODULE_LIST: ${MODULE_LIST}")

	foreach(module ${MODULE_LIST})
		# default
		set(MAIN_DEFAULT MAIN-NOTFOUND)
		set(STACK_DEFAULT 1024)
		set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)

		foreach(property MAIN STACK_MAIN PRIORITY)
			get_target_property(${property} ${module} ${property})
			if(NOT ${property})
				set(${property} ${${property}_DEFAULT})
			endif()
		endforeach()

		if(MAIN)
			# .bdat - { "adis16448", SCHED_PRIORITY_DEFAULT, 2048, adis16448_main },
			set(builtin_apps_string "${builtin_apps_string}{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main },\n")

			# .pdat - int adis16448_main(int argc, char *argv[]);
			set(builtin_apps_decl_string "${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")

			math(EXPR command_count "${command_count}+1")
		endif()
	endforeach()

	configure_file(${PX4_SOURCE_DIR}/cmake/builtins/px4.bdat.in ${OUT}/px4.bdat)
	configure_file(${PX4_SOURCE_DIR}/cmake/builtins/px4.pdat.in ${OUT}/px4.pdat)

endfunction()
