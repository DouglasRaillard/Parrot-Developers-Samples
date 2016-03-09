
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>

#include "_Logging.h"

#define DEFINE_FIELD_NAME(name) #name","
char logging_field_names[] =
"FIELD_TIMESTAMP,"
#include "LoggingFieldNames.h"
;
#undef DEFINE_FIELD_NAME

uint64_t logging_start_timestamp_micro = 0;

FILE *stream;
FIELD_TYPE buffer[FIELD_LAST_ITEM];
LOGGING_BUFFER_STATUS buffer_status = 0;
FIELD_TYPE temp_buffer[sizeof_array(buffer)];

uint64_t _getCurrentTimestampMicro() {
    // Get timestamp in microseconds
    struct timespec tms;
    if (clock_gettime(CLOCK_REALTIME, &tms)) {
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
    return current_timestamp_micro-logging_start_timestamp_micro;
}


LOGGING_BUFFER_STATUS static inline _readBufferStatus() {
    LOGGING_BUFFER_STATUS status;
    __atomic_load(&buffer_status, &status, __ATOMIC_SEQ_CST);
    return status;
}
void static inline _changeBufferStatus(LOGGING_BUFFER_STATUS status) {
    __atomic_store(&buffer_status, &status, __ATOMIC_SEQ_CST);
}

char *getLogFields() {
    return logging_field_names;
}

int initLogging(const char *path) {
    logging_start_timestamp_micro = _getCurrentTimestampMicro();
    stream = fopen(path, "wb");
    if (!stream) {
        return -1;
    }
    // Does not write full length to avoid last comma
    fwrite(logging_field_names, 1, sizeof(logging_field_names)-2, stream);
    fwrite("\n", 1, sizeof("\n"), stream);
    fflush(stream);
    memset(buffer, 0, sizeof(buffer));
    _changeBufferStatus(FIELDS_BUFFER_CLEAN);
    return 0;
}

int shutdownLogging() {
    fclose(stream);
    return 0;
}

int addValueForNextLogEntry(FIELD_TYPE field, FIELD_NAME name) {
    // Must be atomic
    __atomic_store(&buffer[name], &field, __ATOMIC_SEQ_CST);

    // Must be atomic
    _changeBufferStatus(FIELDS_BUFFER_DIRTY);
    return 0;
}

int __dumpLog(FILE* stream, uint64_t current_timestamp_micro) {
    if(!stream) {
        return -1;
    }

    // Must be atomic
    if(_readBufferStatus() != FIELDS_BUFFER_DIRTY) {
        return 0;
    }

    // Must be atomic
    _changeBufferStatus(FIELDS_BUFFER_CLEAN);

    // Make our copy
    for(size_t i=0; i < sizeof_array(temp_buffer); i++) {
        // Must be atomic
        __atomic_store(&temp_buffer[i], &buffer[i], __ATOMIC_SEQ_CST);
    }

    // Use 19 char because this is the biggest field that can be stored in uint64_t, +1 for the comma
    char timestamp_buffer[19+1];
    int string_length = snprintf(timestamp_buffer, sizeof(timestamp_buffer), "%lu,", current_timestamp_micro);
    fwrite(timestamp_buffer, 1, string_length, stream);

    // Start from the second item because we handle timestamp appart
    for(size_t i=1; i < sizeof_array(temp_buffer); i++) {
        char field_buffer[FIELDS_PRECISION_INTEGRAL+1+FIELDS_PRECISION_DECIMAL+1]; // digits + the dot + null character
        int string_length = snprintf(field_buffer, sizeof(field_buffer), "%"FIELDS_PRECISION_INTEGRAL_STRING"."FIELDS_PRECISION_DECIMAL_STRING"f", (double)temp_buffer[i]);
        fwrite(field_buffer, 1, string_length, stream);
        // Print separator only if not last field on the line
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

    // Get timestamp in microseconds
    uint64_t current_timestamp_micro = _getCurrentTimestampMicro();

    // If it is time to dump the fields
    if (current_timestamp_micro >= (last_timestamp_micro + DUMP_PERIOD_MICRO)) {
        __dumpLog(stream, current_timestamp_micro);
        last_timestamp_micro = current_timestamp_micro;
    } else {
        return 1;
    }
    return 0;
}

