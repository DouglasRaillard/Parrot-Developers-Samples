#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>
#include "redtracking.h"

using namespace cv;
using namespace std;

enum EnableTracking{OBJECT_DETECTED,OBJECT_NOT_DETECTED};


int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

static EnableTracking trackingStatus = OBJECT_NOT_DETECTED;
static std::vector<cv::Vec4i> hierarchy;
static std::vector<std::vector<cv::Point> > contours;
static std::vector<std::vector<cv::Point> > contours_poly;
static std::vector<cv::Rect> targetZone;

static ARSAL_Thread_t redtracking_thread = NULL;
static MEASURED_DATA_T measured_data_buffer;
static pthread_mutex_t measured_data_lock;


void callbackButton(EnableTracking &trackingStatus)
{
    if(trackingStatus == OBJECT_DETECTED)
    {
        trackingStatus = OBJECT_NOT_DETECTED;
    }
    else
    {
        trackingStatus = OBJECT_DETECTED;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

int init_redtracking() {
    namedWindow("Autopilote Target Setter", CV_WINDOW_AUTOSIZE); //create a window called "Control"


    //Create trackbars in "Autopilote Target Setter" window
    cvCreateTrackbar("LowH", "Autopilote Target Setter", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Autopilote Target Setter", &iHighH, 179);

    cvCreateTrackbar("LowS", "Autopilote Target Setter", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Autopilote Target Setter", &iHighS, 255);

    cvCreateTrackbar("LowV", "Autopilote Target Setter", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Autopilote Target Setter", &iHighV, 255);

    //cvCreateButton("toogle tracking",callbackButton(trackingStatus),NULL,CV_PUSH_BUTTON,1);

    // Display the window
    //waitKey(1);

    pthread_mutex_init(&measured_data_lock, NULL);

    // Create OpenCV thread
    ARSAL_Thread_Create(&redtracking_thread, redtracking_thread_loop, NULL);
}


void *redtracking_thread_loop(void* data) {
    VideoCapture cap = VideoCapture("./video_fifo.h264");
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));


    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the H.264 stream from named pipe" << endl;
        exit(-1);
    }

    Mat imgOriginal;

    while(true) {
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            continue;
        }

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        if(trackingStatus == OBJECT_DETECTED)
        {
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            cv::findContours(imgThresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));

            contours_poly.clear();
            contours_poly.resize(contours.size());

            targetZone.clear();
            targetZone.resize(contours.size());

            for(size_t j=0;j<contours.size();j++)
            {
                cv::approxPolyDP(cv::Mat(contours[j]), contours_poly[j], 3, true);
                targetZone[j] = cv::boundingRect(cv::Mat(contours_poly[j]));
                cv::rectangle(imgOriginal, targetZone[j], cv::Scalar( 0, 0, 255), 2, 8, 0 );
            }
        }

        imshow("Thresholded Image", imgThresholded);
        imshow("Original", imgOriginal); //show the original image

        waitKey(25);
    }
    return 0;
}




MEASURED_DATA_T redtracking_get_measured_data() {
    MEASURED_DATA_T temp;
    pthread_mutex_lock(&measured_data_lock);
    temp = measured_data_buffer;
    pthread_mutex_unlock(&measured_data_lock);
    return temp;

}


void redtracking_update_measured_data(MEASURED_DATA_T* data) {
    pthread_mutex_lock(&measured_data_lock);
    measured_data_buffer = *data;
    pthread_mutex_unlock(&measured_data_lock);
}



#ifdef __cplusplus
}
#endif
