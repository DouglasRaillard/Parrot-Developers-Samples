
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>

#include "_Logger.h"

#define DEFINE_VALUE_NAME(name) #name","
char logger_value_names[] =
#include "LoggerValueNames.h"
;
#undef DEFINE_VALUE_NAME

VALUES_TYPE buffer[VALUE_LAST_ITEM+1];
VALUES_TYPE temp_buffer[sizeof_array(buffer)];
FILE *stream;


int initLogger(const char *path) {
    stream = fopen(path, "wb");
    if (!stream) {
        return -1;
    }
    fwrite(logger_value_names, sizeof(logger_value_names[0]), sizeof(logger_value_names)-2, stream);
    fwrite("\n", 1, 1, stream);
    fflush(stream);
    memset(buffer, 0, sizeof(buffer));
    buffer[VALUE_LAST_ITEM] = VALUES_BUFFER_CLEAN;
    return 0;
}

int shutdownLogger() {
    fclose(stream);
    return 0;
}

int addValueForNextLogEntry(VALUES_TYPE value, VALUES_NAME name) {
    // Must be atomic
    __atomic_store(&buffer[name], &value, __ATOMIC_SEQ_CST);
    //buffer[name] = value;

    // Must be atomic
    value = VALUES_BUFFER_DIRTY;
    __atomic_store(&buffer[VALUE_LAST_ITEM], &value, __ATOMIC_SEQ_CST);
    //buffer[VALUE_LAST_ITEM] = VALUES_BUFFER_DIRTY;
    return 0;
}

int __dumpLog() {
    // Must be atomic
    {
        VALUES_TYPE value_temp;
        __atomic_load(&buffer[VALUE_LAST_ITEM], &value_temp, __ATOMIC_SEQ_CST);
        if(value_temp != VALUES_BUFFER_DIRTY) {
        //if(buffer[VALUE_LAST_ITEM] != VALUES_BUFFER_DIRTY) {
            return 0;
        }
    }

    if(!stream) {
        return -1;
    }

    // Must be atomic
    {
        VALUES_TYPE value = VALUES_BUFFER_CLEAN;
        __atomic_store(&buffer[VALUE_LAST_ITEM], &value, __ATOMIC_SEQ_CST);
        //buffer[VALUE_LAST_ITEM] = VALUES_BUFFER_CLEAN;
    }
    // Make our copy
    memcpy(temp_buffer, buffer, sizeof(temp_buffer));


    for(size_t i=0; i < sizeof_array(temp_buffer); i++) {
        char value_buffer[VALUES_PRECISION_INTEGRAL+1+VALUES_PRECISION_DECIMAL+1]; // digits + the dot + null character
        int string_length = snprintf(value_buffer, sizeof(value_buffer), "%"VALUES_PRECISION_INTEGRAL_STRING"."VALUES_PRECISION_DECIMAL_STRING"f", (double)temp_buffer[i]);
        fwrite(value_buffer, sizeof(value_buffer[0]), string_length+1, stream);
        // Print separator only if not last value on the line
        if(i+1 != sizeof_array(temp_buffer)) {
            fwrite(SEPARATOR_STRING, sizeof(char), sizeof(SEPARATOR_STRING), stream);
        }
    }
    fwrite("\n", sizeof(char), sizeof("\n"), stream);
    fflush(stream);

    return 0;
}

int dumpLog() {
    static uint64_t last_timestamp_micro = 0;
    if(!stream) {
        return -1;
    }

    // Get timestamp in microseconds
    struct timespec tms;
    if (clock_gettime(CLOCK_REALTIME,&tms)) {
        return -2;
    }
    /* seconds, multiplied with 1 million */
    uint64_t current_timestamp_micro = tms.tv_sec * 1000000;
    /* Add full current_timestamp_microeconds */
    current_timestamp_micro += tms.tv_nsec/1000;
    /* round up if necessary */
    if (tms.tv_nsec % 1000 >= 500) {
        ++current_timestamp_micro;
    }

    // If it is time to dump the values
    if (current_timestamp_micro >= (last_timestamp_micro + DUMP_PERIOD_MICRO)) {
        __dumpLog();
        last_timestamp_micro = current_timestamp_micro;
    }
    return 0;
}

