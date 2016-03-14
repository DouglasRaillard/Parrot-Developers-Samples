#ifndef MEASURED_DATA_H_
#define MEASURED_DATA_H_
#ifdef __cplusplus
extern "C" {
#endif

typedef struct MEASURED_DATA_T {
    long x1;
    long x2;
} MEASURED_DATA_T;

MEASURED_DATA_T redtracking_get_measured_data();
void redtracking_update_measured_data(MEASURED_DATA_T* data);


#ifdef __cplusplus
}
#endif
#endif //MEASURED_DATA_H_
