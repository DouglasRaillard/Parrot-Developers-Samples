#ifndef MEASURED_DATA_H_
#define MEASURED_DATA_H_

#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MEASURED_DATA_T {
    std::vector<std::pair<double,double> > centers;
    std::vector<double> areas;

} MEASURED_DATA_T;

// Get data created by the OpenCV thread
MEASURED_DATA_T redtracking_get_measured_data();

// Update the data (use it from the OpenCV thread)
void redtracking_update_measured_data(MEASURED_DATA_T* data);


#ifdef __cplusplus
}
#endif
#endif //MEASURED_DATA_H_
