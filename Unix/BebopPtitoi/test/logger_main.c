#include "../Logger.h"

int main() {
    initLogger("./dump");
    printf("hello %i\n", VALUE_LAST_ITEM);
    addValueForNextLogEntry(33.4, VALUE_PITCH);
    while(1) {
        addValueForNextLogEntry(88.4, VALUE_YAW);
        dumpLog();
    }
    shutdownLogger();
}
