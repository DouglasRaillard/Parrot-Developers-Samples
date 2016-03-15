#ifndef REDTRACKING_H_
#define REDTRACKING_H_
#include "MeasuredData.h"

#ifdef __cplusplus
extern "C" {
#endif

// DRAGONS HERE: include all C headers insied extern "C"
#include <libARSAL/ARSAL.h>

// SHOULD BE CLEANED: this spinlock is from BebopPiloting.c
// Spinlock used to stop the output of frames in the named pipe
extern pthread_spinlock_t video_frame_lock;

// Init OpenCV
int init_redtracking();

// Thread for OpenCV computation
void *redtracking_thread_loop(void* data);

#ifdef __cplusplus
}
#endif
#endif //REDTRACKING_H_
