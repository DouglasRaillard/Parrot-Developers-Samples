#include <stdio.h>
#include "_LoggingFieldNamesEnum.h"

#define DUMP_PERIOD_MICRO 1E6
#define FIELDS_PRECISION_DECIMAL 5
#define FIELDS_PRECISION_INTEGRAL 3
#define SEPARATOR_STRING ","
typedef double FIELD_TYPE;

int dumpLog();
int initLogging(const char *path);
int shutdownLogging();
int addValueForNextLogEntry(FIELD_TYPE field, FIELD_NAME name);
char *getLogFields();
