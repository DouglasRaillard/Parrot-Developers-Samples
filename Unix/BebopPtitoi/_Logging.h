
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>

#include "Logging.h"

#define __STRINGIFY(...) #__VA_ARGS__
#define _STRINGIFY(string) __STRINGIFY(string)
#define STRINGIFY(string) _STRINGIFY(string)


#define FIELDS_PRECISION_INTEGRAL_STRING STRINGIFY(FIELDS_PRECISION_INTEGRAL)
#define FIELDS_PRECISION_DECIMAL_STRING STRINGIFY(FIELDS_PRECISION_DECIMAL)
#define sizeof_array(array) (sizeof(array)/sizeof(array[0]))


typedef enum LOGGING_BUFFER_STATUS {
    FIELDS_BUFFER_CLEAN, // The buffer does not need to be dumped
    FIELDS_BUFFER_DIRTY // The buffer has been modified and needs to be dumped
} LOGGING_BUFFER_STATUS;

