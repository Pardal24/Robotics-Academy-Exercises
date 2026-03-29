#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

int main() {

    VideoCapture cap(0, CAP_V4L2);

    if (!cap.isOpened()) {
        std::cout << "Camera failed" << std::endl;
        return -1;
    }

    Mat frame;
    
    Scalar low_red1(0,120,70);
    Scalar upper_red1(10,255,255);

    Scalar low_red2(170,120,70);
    Scalar upper_red2(179,255,255);

    
    while(true) {
        cap.read(frame);

        if(frame.empty()) break;

        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        Mat mask1, mask2;
        inRange(hsv, low_red1, upper_red1, mask1 );
        inRange(hsv, low_red2, upper_red2, mask2);

        Mat mask;
        bitwise_or(mask1,mask2,mask);

        //morphological opening -> Removes noise
        erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        //morphological closing -> Fixes object shape
        dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        Moments img_moments = moments(mask);

        double m00 = img_moments.m00;
        double m10 = img_moments.m10;
        double m01 = img_moments.m01;

        Mat mask_color;
        cvtColor(mask, mask_color, COLOR_GRAY2BGR);

        if (m00 > 10000){

            int ctr_x = m10/m00;
            int ctr_y = m01/m00;

            circle(mask_color, Point(ctr_x,ctr_y), 5, Scalar(0,0,255), -1);

        }

        imshow("Camera", frame);
        imshow("Mask", mask_color);

        if(waitKey(1) == 27) break;
    }

    return 0;
}