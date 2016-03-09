#include <stdio.h>
#include "_LoggerValueNamesType.h"

#define DUMP_PERIOD_MICRO 1E6
#define VALUES_PRECISION_DECIMAL 5
#define VALUES_PRECISION_INTEGRAL 3
#define SEPARATOR_STRING ","
typedef double VALUES_TYPE;

int dumpLog();
int initLogger(const char *path);
int shutdownLogger();
int addValueForNextLogEntry(VALUES_TYPE value, VALUES_NAME name);
