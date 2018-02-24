#include "StateMachine.h"

StateMachine::StateMachine()
	: isAcceptable(false)
{
	setDebugFlag(true);
}

bool StateMachine::CheckString(const char *theString)
{
	enterStartState();

	EOS();

	return isAcceptable;
}
