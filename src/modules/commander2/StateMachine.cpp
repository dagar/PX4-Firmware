#include "StateMachine.h"

StateMachine::StateMachine()
	: isAcceptable(false)
{
	setDebugFlag(true);
}

bool StateMachine::CheckString(const char *theString)
{
	enterStartState();

	while (*theString != '\0') {
		switch (*theString) {
		case '0':
			Zero();
			break;

		case '1':
			One();
			break;

		default:
			Unknown();
			break;
		}

		++theString;
	}

	// end of string has been reached - send the EOS transition.
	EOS();

	return isAcceptable;
}
