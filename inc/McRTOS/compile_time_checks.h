/**
 * @file compile_time_checks.h
 *
 * Compile-time and static-analysis checking macros
 *
 * @author German Rivera 
 */ 
#ifndef _COMPILE_TIME_CHECKS_H
#define _COMPILE_TIME_CHECKS_H

/*
 * Compile-time assertion macros
 */ 

#define C_ASSERT(_cond) \
        extern const char c_assert_dummy_decl[(_cond) ? 1 : -1]

#define C_ASSERT2(_assertName, _cond) \
        typedef char c_assert__ ## _assertName[(_cond) ? 1 : -1]

/*
 * Annotation macros to be used to "annotate" function parameters
 * and typedefs. These annotations can be used by a static analysis tool.
 */ 

/**
 * Untrusted input parameter
 */
#define _UNTRUSTED_IN_

/**
 * Sanitized (or trusted) input parameter
 */
#define _IN_

/**
 * Output parameter
 */
#define _OUT_

/**
 * Input/Output parameter
 */
#define _INOUT_

/**
 * Unused parameter
 */
#define _UNUSED_

/**
 * Integer subrange
 */
#define _RANGE_(_min, _max)

/**
 * Annotation to be placed in front of the prototype of a function
 * that requires the caller to ensure mutual exclusion, when invoked
 * from multiple threads
 */
#define _REQUIRES_MUTUAL_EXCLUSION_

/**
 * Annotations to be placed in front of the prototype of a function
 * that is not supposed to return to its caller, ever or some times.
 */
#define _NEVER_RETURN_
#define _NEVER_RETURN_ON_SUCCESS_
#define _NEVER_RETURN_ON_ERROR_
#define _MAY_NOT_RETURN_
#define _MAY_NOT_RETURN_ON_SUCCESS_
#define _MAY_NOT_RETURN_ON_ERROR_

/**
 * Annotations to be placed in front of the prototype of a function
 * that is supposed called only from threads and not from interrupt
 * handlers.
 */
#define _THREAD_CALLERS_ONLY_

/**
 * Macro to give a hint to the compiler for static branch prediction
 */
#define _INFREQUENTLY_TRUE_(_condition) \
        __builtin_expect((_condition), 0)

/**
 * Macro to generate a message pragma as a TODO reminder 
 */
#define TODO(_todo_msg) __DO_PRAGMA(message ("*** TODO: " #_todo_msg))
#define __DO_PRAGMA(_x) _Pragma(#_x)

#endif /* _COMPILE_TIME_CHECKS_H */

