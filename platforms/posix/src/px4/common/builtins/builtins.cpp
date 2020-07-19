

#include "builtins.h"

#include <string.h>
#include <limits.h>
#include <errno.h>

const struct builtin_s *g_builtins;
int g_builtin_count;

const char *builtin_getname(int index)
{
	const struct builtin_s *builtin = builtin_for_index(index);

	if (builtin != nullptr) {
		return builtin->name;
	}

	return nullptr;
}

int builtin_isavail(const char *appname)
{
	const char *name = nullptr;

	for (int i = 0; (name = builtin_getname(i)) != nullptr; i++) {
		if (strncmp(name, appname, NAME_MAX) == 0) {
			return i;
		}
	}

	return -ENOENT;
}

const struct builtin_s *builtin_for_index(int index)
{
	if (index < g_builtin_count) {
		return &g_builtins[index];
	}

	return nullptr;
}
