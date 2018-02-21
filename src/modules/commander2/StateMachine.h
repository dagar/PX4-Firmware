#ifndef _H_THECONTEXT
#define _H_THECONTEXT

#include "StateMachine_sm.h"

class StateMachine : public StateMachineContext<StateMachine>
{
private:

	bool isAcceptable;
	// If a string is acceptable, then this variable is set to YES;
	// NO, otherwise.

public:
	StateMachine();
	// Default constructor.

	~StateMachine() {};
	// Destructor.

	bool CheckString(const char *);
	// Checks if the string is acceptable.

	inline void Acceptable()
	{ isAcceptable = true; };

	inline void Unacceptable()
	{ isAcceptable = false; };
	// State map actions.
};

#endif
