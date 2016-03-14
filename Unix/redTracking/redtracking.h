#ifndef REDTRACKING_H_
#define REDTRACKING_H_

#ifdef __cplusplus
extern "C" {
#endif

// DRAGONS HERE: include all C headers insied extern "C"
#include "MeasuredData.h"
#include <libARSAL/ARSAL.h>


// Init OpenCV
int init_redtracking();

// Thread for OpenCV computation
void *redtracking_thread_loop(void* data);

#ifdef __cplusplus
}
#endif
#endif //REDTRACKING_H_
