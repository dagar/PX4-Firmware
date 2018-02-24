
#include "StateMachine.h"

#include <px4_defines.h>

extern "C" __EXPORT int commander2_main(int argc, char *argv[]);

int commander2_main(int argc, char *argv[])
{
	StateMachine thisContext;


	if (argc < 2) {
		PX4_INFO("No string to check.");

	} else if (argc > 2) {
		PX4_INFO("Only one argument is accepted.");

	} else {
		if (thisContext.CheckString(argv[1]) == false) {
			PX4_INFO("not acceptable.");

		} else {
			PX4_INFO("acceptable.");
		}
	}

	return 0;
}
