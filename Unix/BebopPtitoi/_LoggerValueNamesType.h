
#define DEFINE_VALUE_NAME(name) name,
typedef enum VALUES_NAME {
#include "LoggerValueNames.h"
    VALUE_LAST_ITEM // MUST ALWAYS BE THE LAST IN THE LIST
                    // Also used to store the state of the buffer
} VALUES_NAME;
#undef DEFINE_VALUE_NAME
