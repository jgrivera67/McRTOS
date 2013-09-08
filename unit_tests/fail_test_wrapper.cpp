///
/// @file fail_test_wrapper.cpp
///
/// Wrapper to call the CPPUTEST FAIL_TEST macro from the product code being tested.
///
/// @author: German Rivera
///

#include <stdio.h>
#include <stdarg.h>
#include "CppUTest/TestHarness.h"


/**
 * Wrapper to fail a test from the product code being tested, due to
 * an FDC_ASSER failure.
 *
 * @param fmt               format string
 *
 * @return None
 */
extern "C"
void
cpputest_fail_test_fdc_assert(char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    vfprintf(stderr, fmt, va);
    va_end(va);

    FAIL_TEST("FDC_ASSERT failed");
}

