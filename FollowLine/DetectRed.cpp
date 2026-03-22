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

    while(true) {
        cap.read(frame);

        if(frame.empty()) break;

        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        Scalar low_red1(0,120,70);
        Scalar upper_red1(10,255,255);

        Scalar low_red2(170,120,70);
        Scalar upper_red2(179,255,255);

        Mat mask1, mask2;
        inRange(hsv, low_red1, upper_red1, mask1 );
        inRange(hsv, low_red2, upper_red2, mask2);

        Mat mask;
        bitwise_or(mask1,mask2,mask);

        imshow("Camera", frame);
        imshow("Mask",mask);

        if(waitKey(1) == 27) break;
    }

    return 0;
}