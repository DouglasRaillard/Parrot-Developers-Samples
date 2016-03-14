#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

enum EnableTracking{OBJECT_DETECTED,OBJECT_NOT_DETECTED};

VideoCapture cap; //capture the video from web cam

int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

EnableTracking trackingStatus = OBJECT_NOT_DETECTED;
std::vector<cv::Vec4i> hierarchy;
std::vector<std::vector<cv::Point> > contours;
std::vector<std::vector<cv::Point> > contours_poly;
std::vector<cv::Rect> targetZone;

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

int _init_redtracking() {
    //cap = VideoCapture("./video_fifo.h264");
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));

    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        //return -1;
    }

    namedWindow("Autopilote Target Setter", CV_WINDOW_AUTOSIZE); //create a window called "Control"


    //Create trackbars in "Autopilote Target Setter" window
    cvCreateTrackbar("LowH", "Autopilote Target Setter", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Autopilote Target Setter", &iHighH, 179);

    cvCreateTrackbar("LowS", "Autopilote Target Setter", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Autopilote Target Setter", &iHighS, 255);

    cvCreateTrackbar("LowV", "Autopilote Target Setter", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Autopilote Target Setter", &iHighV, 255);

    //cvCreateButton("toogle tracking",callbackButton(trackingStatus),NULL,CV_PUSH_BUTTON,1);
}

#ifdef __cplusplus
extern "C" {
#endif
int init_redtracking() {
    return _init_redtracking();
}
#ifdef __cplusplus
}
#endif

int _redtracking_frame_callback()
{
    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
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

        if (waitKey(30) == 27)
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;

}



#ifdef __cplusplus
extern "C" {
#endif
int redtracking_frame_callback() {
    return _redtracking_frame_callback();
}
#ifdef __cplusplus
}
#endif
