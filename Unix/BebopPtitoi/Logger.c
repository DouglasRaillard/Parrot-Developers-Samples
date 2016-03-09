
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>

#include "_Logger.h"

#define DEFINE_VALUE_NAME(name) #name","
char logger_value_names[] =
"VALUE_TIMESTAMP,"
#include "LoggerValueNames.h"
;
#undef DEFINE_VALUE_NAME

FILE *stream;
VALUES_TYPE buffer[VALUE_LAST_ITEM];
VALUES_BUFFER_STATUS buffer_status = 0;
VALUES_TYPE temp_buffer[sizeof_array(buffer)];

VALUES_BUFFER_STATUS static inline _readBufferStatus() {
    VALUES_BUFFER_STATUS status;
    __atomic_load(&buffer_status, &status, __ATOMIC_SEQ_CST);
    return status;
}
void static inline _changeBufferStatus(VALUES_BUFFER_STATUS status) {
    __atomic_store(&buffer_status, &status, __ATOMIC_SEQ_CST);
}

int initLogger(const char *path) {
    stream = fopen(path, "wb");
    if (!stream) {
        return -1;
    }
    // Does not write full length to avoid last comma
    fwrite(logger_value_names, 1, sizeof(logger_value_names)-2, stream);
    fwrite("\n", 1, sizeof("\n"), stream);
    fflush(stream);
    memset(buffer, 0, sizeof(buffer));
    _changeBufferStatus(VALUES_BUFFER_CLEAN);
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
    _changeBufferStatus(VALUES_BUFFER_DIRTY);
    return 0;
}

int __dumpLog(uint64_t current_timestamp_micro) {
    if(!stream) {
        return -1;
    }

    // Must be atomic
    if(_readBufferStatus() != VALUES_BUFFER_DIRTY) {
        return 0;
    }

    // Must be atomic
    _changeBufferStatus(VALUES_BUFFER_CLEAN);

    // Make our copy
    for(size_t i=0; i < sizeof_array(temp_buffer); i++) {
        __atomic_store(&temp_buffer[i], &buffer[i], __ATOMIC_SEQ_CST);
    }

    // Use 19 char because this is the biggest value that can be stored in uint64_t, +1 for the comma
    char timestamp_buffer[19+1];
    int string_length = snprintf(timestamp_buffer, sizeof(timestamp_buffer), "%lu,", current_timestamp_micro);
    fwrite(timestamp_buffer, 1, string_length, stream);

    // Start from the second item because we handle timestamp appart
    for(size_t i=1; i < sizeof_array(temp_buffer); i++) {
        char value_buffer[VALUES_PRECISION_INTEGRAL+1+VALUES_PRECISION_DECIMAL+1]; // digits + the dot + null character
        int string_length = snprintf(value_buffer, sizeof(value_buffer), "%"VALUES_PRECISION_INTEGRAL_STRING"."VALUES_PRECISION_DECIMAL_STRING"f", (double)temp_buffer[i]);
        fwrite(value_buffer, 1, string_length, stream);
        // Print separator only if not last value on the line
        if(i+1 != sizeof_array(temp_buffer)) {
            fwrite(SEPARATOR_STRING, 1, sizeof(SEPARATOR_STRING), stream);
        }
    }
    fwrite("\n", 1, sizeof("\n"), stream);
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
        __dumpLog(current_timestamp_micro);
        last_timestamp_micro = current_timestamp_micro;
    }
    return 0;
}

