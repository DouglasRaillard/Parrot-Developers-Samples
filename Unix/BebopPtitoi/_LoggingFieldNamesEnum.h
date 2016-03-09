
#define DEFINE_FIELD_NAME(name) name,
typedef enum FIELD_NAME {
    FIELD_TIMESTAMP,
#include "LoggingFieldNames.h"
    FIELD_LAST_ITEM // MUST ALWAYS BE THE LAST IN THE LIST
                    // Also used to store the state of the buffer
} FIELD_NAME;
#undef DEFINE_FIELD_NAME
