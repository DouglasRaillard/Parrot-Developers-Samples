
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>

#include "Logger.h"

#define __STRINGIFY(...) #__VA_ARGS__
#define _STRINGIFY(string) __STRINGIFY(string)
#define STRINGIFY(string) _STRINGIFY(string)


#define VALUES_PRECISION_INTEGRAL_STRING STRINGIFY(VALUES_PRECISION_INTEGRAL)
#define VALUES_PRECISION_DECIMAL_STRING STRINGIFY(VALUES_PRECISION_DECIMAL)
#define sizeof_array(array) (sizeof(array)/sizeof(array[0]))


typedef enum VALUES_BUFFER_STATUS {
    VALUES_BUFFER_CLEAN, // The buffer does not need to be dumped
    VALUES_BUFFER_DIRTY // The buffer has been modified and needs to be dumped
} VALUES_BUFFER_STATUS;

