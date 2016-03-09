#include "../Logging.h"

int main() {
    initLogging("./dump");
    addValueForNextLogEntry(33.4, FIELD_PITCH);
    while(1) {
        addValueForNextLogEntry(88.4, FIELD_YAW);
        dumpLog();
    }
    shutdownLogging();
}
