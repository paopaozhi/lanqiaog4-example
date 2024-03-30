#ifndef __TEST_H
#define __TEST_H

#include <stdio.h>

#ifdef TEST
#define test_info(format, ...) \
        printf("[test info] "format"\n", ##__VA_ARGS__)
#else
#define test_info(format, ...)
#endif
 
#endif
