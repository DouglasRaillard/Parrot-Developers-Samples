#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>
#include <cstdio>
#include "redtracking.h"

using namespace cv;
using namespace std;

enum EnableTracking{OBJECT_DETECTED,OBJECT_NOT_DETECTED};


static int iLowH = 0;
static int iHighH = 179;

static int iLowS = 0;
static int iHighS = 255;

static int iLowV = 0;
static int iHighV = 255;

static int tracking = 0;
static int nbTargets = 1;

static EnableTracking trackingStatus = OBJECT_NOT_DETECTED;
static std::vector<cv::Vec4i> hierarchy;
static std::vector<std::vector<cv::Point> > contours;
static std::vector<std::vector<cv::Point> > contours_poly;
static std::vector<cv::Rect> targetZone;
static std::vector<cv::Rect> target;
static std::vector<cv::Point> centers;

static ARSAL_Thread_t redtracking_thread = NULL;
static MEASURED_DATA_T measured_data_buffer;
static pthread_mutex_t measured_data_lock;


bool compare_rect(const Rect &a, const Rect &b)
{
    return a.area() < b.area();
}

void defineTarget(std::vector<cv::Rect> potentialTargets, std::vector<cv::Rect> &target, int nbTargets)
{
    target.clear();
    if (potentialTargets.empty()) {
        return;
    }

    std::sort(potentialTargets.begin(), potentialTargets.end(), compare_rect);

    if(nbTargets >= potentialTargets.size())
    {
        nbTargets = potentialTargets.size()-1;
    }

    for(std::vector<cv::Rect>::iterator it = potentialTargets.end()-nbTargets; it != potentialTargets.end(); it++)
    {
        target.push_back(*it);
    }
}

void defineCenter(std::vector<cv::Rect> target, std::vector<cv::Point> &centers)
{
    cv::Point currentPoint;
    centers.clear();

    for(std::vector<cv::Rect>::iterator it = target.begin(); it != target.end(); it++)
    {
        currentPoint.x = (*it).x + (*it).width/2;
        currentPoint.y = (*it).y + (*it).height/2;

        centers.push_back(currentPoint);
    }
}


#ifdef __cplusplus
extern "C" {
#endif

int init_redtracking() {
    pthread_mutex_init(&measured_data_lock, NULL);

    // Create OpenCV thread
    ARSAL_Thread_Create(&redtracking_thread, redtracking_thread_loop, NULL);
}


void *redtracking_thread_loop(void* data) {
    namedWindow("Autopilote Target Setter", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Autopilote Target Setter" window
    cvCreateTrackbar("LowH", "Autopilote Target Setter", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Autopilote Target Setter", &iHighH, 179);

    cvCreateTrackbar("LowS", "Autopilote Target Setter", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Autopilote Target Setter", &iHighS, 255);

    cvCreateTrackbar("LowV", "Autopilote Target Setter", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Autopilote Target Setter", &iHighV, 255);

    cvCreateTrackbar("nbTrackedOjects", "Autopilote Target Setter", &nbTargets, 50); //tracking (0 - 1)
    cvCreateTrackbar("Tracking", "Autopilote Target Setter", &tracking, 1); //tracking (0 - 1)

    VideoCapture cap = VideoCapture("./video_fifo.h264");
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));

    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the H.264 stream from named pipe" << endl;
        exit(-1);
    }

    // Flush the named pipe
    FILE *fp = fopen("./video_fifo.h264", "rb");
    fseek(fp, 0, SEEK_END);
    size_t fsize = ftell(fp);
    rewind(fp);
    char temp[4096];
    for(size_t i=0; i<(fsize/sizeof(temp)); i++){
        fread(&temp, 1, sizeof(temp), fp);
    }
    fclose(fp);


    Mat imgOriginal;

    while(true) {
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            continue;
        }

        Mat imgHSV;
        measured_data_buffer.centers.clear();

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        if(tracking)
        {
            trackingStatus = OBJECT_DETECTED;
        }
        else
        {
            trackingStatus = OBJECT_NOT_DETECTED;
        }

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
            }

            defineTarget(targetZone, target, nbTargets);
            defineCenter(target, centers);

            if(contours.size() != 0)
            {
                for(size_t i=0; i<target.size(); i++)
                {
                    cv::rectangle(imgOriginal, target[i], cv::Scalar( 0, 0, 255), 2, 8, 0 );
                    cv::circle(imgOriginal, centers[i], 1, cv::Scalar(0,255,0), 7, 24);
                    cv::putText(imgOriginal, "["+std::to_string(centers[i].x).substr(0,5)+";"+std::to_string(centers[i].y).substr(0,5)+"]", centers[i], cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

                    //update infos for ptitoi/picot command loop
                    measured_data_buffer.centers.push_back(std::make_pair(centers[i].x, centers[i].y));
                    redtracking_update_measured_data(&measured_data_buffer);
                }
            }
        }

        imshow("Thresholded Image", imgThresholded);
        imshow("Original", imgOriginal); //show the original image

        if(waitKey(25) == 27)
        {
            cout << "esc key is pressed by user" << endl;
            exit(0);
        }
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
