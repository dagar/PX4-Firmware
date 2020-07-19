


#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct builtin_s {
	const char *name;         /* Invocation name and as seen under /sbin/ */
	int         priority;     /* Use: SCHED_PRIORITY_DEFAULT */
	int         stacksize;    /* Desired stack size */
	main_t      main;         /* Entry point: main(int argc, char *argv[]) */
};

EXTERN const struct builtin_s g_builtins[];
EXTERN const int g_builtin_count;


int builtin_isavail(const char *appname);
const char *builtin_getname(int index);

const struct builtin_s *builtin_for_index(int index);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
