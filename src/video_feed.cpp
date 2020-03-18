#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include<bits/stdc++.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;
using namespace ros;


int main(int argc, char **argv){
    init(argc, argv, "camera");
    NodeHandle n;
    VideoCapture cap(0);
    if(!cap.isOpened())
	    return -1;
    image_transport::ImageTransport it(n);
    image_transport::Publisher video_pub = it.advertise("videofeed", 1);
    Rate loop_rate(10);
    while(n.ok()){
	    Mat img;
	    cap>>img;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        video_pub.publish(msg);
        namedWindow("WebCam", WINDOW_NORMAL);
	    imshow("WebCam",img);
    	waitKey(100);
        spinOnce();
        loop_rate.sleep();
    }
    return 0;
}