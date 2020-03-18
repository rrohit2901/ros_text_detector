#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/dnn.hpp>
#include "ros/ros.h"
#include<bits/stdc++.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include<sstream>


using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace ros;

void decode(const Mat& scores, const Mat& geometry, float scoreThresh, std::vector<RotatedRect>& detections, std::vector<float>& confidences){
    detections.clear();
    CV_Assert(scores.dims == 4); CV_Assert(geometry.dims == 4); CV_Assert(scores.size[0] == 1);
    CV_Assert(geometry.size[0] == 1); CV_Assert(scores.size[1] == 1); CV_Assert(geometry.size[1] == 5);
    CV_Assert(scores.size[2] == geometry.size[2]); CV_Assert(scores.size[3] == geometry.size[3]);

    const int height = scores.size[2];
    const int width = scores.size[3];
    for (int y = 0; y < height; ++y)
    {
        const float* scoresData = scores.ptr<float>(0, 0, y);
        const float* x0_data = geometry.ptr<float>(0, 0, y);
        const float* x1_data = geometry.ptr<float>(0, 1, y);
        const float* x2_data = geometry.ptr<float>(0, 2, y);
        const float* x3_data = geometry.ptr<float>(0, 3, y);
        const float* anglesData = geometry.ptr<float>(0, 4, y);
        for (int x = 0; x < width; ++x)
        {
            float score = scoresData[x];
            if (score < scoreThresh)
                continue;

            // Decode a prediction.
            // Multiple by 4 because feature maps are 4 time less than input image.
            float offsetX = x * 4.0f, offsetY = y * 4.0f;
            float angle = anglesData[x];
            float cosA = std::cos(angle);
            float sinA = std::sin(angle);
            float h = x0_data[x] + x2_data[x];
            float w = x1_data[x] + x3_data[x];

            Point2f offset(offsetX + cosA * x1_data[x] + sinA * x2_data[x],
                           offsetY - sinA * x1_data[x] + cosA * x2_data[x]);
            Point2f p1 = Point2f(-sinA * h, -cosA * h) + offset;
            Point2f p3 = Point2f(-cosA * w, sinA * w) + offset;
            RotatedRect r(0.5f * (p1 + p3), Size2f(w, h), -angle * 180.0f / (float)CV_PI);
            detections.push_back(r);
            confidences.push_back(score);
        }
    }
}

string detection_east(Mat img){
    static const std::string kWinName = "EAST: An Efficient and Accurate Scene Text Detector";
    namedWindow(kWinName, WINDOW_NORMAL);

    std::vector<Mat> outs;
    std::vector<String> outNames(2);
    outNames[0] = "feature_fusion/Conv_7/Sigmoid";
    outNames[1] = "feature_fusion/concat_3";
    blobFromImage(frame, blob, 1.0, Size(inpWidth, inpHeight), Scalar(123.68, 116.78, 103.94), true, false);
    net.setInput(blob);
    net.forward(outs, outNames);
    Mat scores = outs[0];
    Mat geometry = outs[1];

    // Decode predicted bounding boxes.
    std::vector<RotatedRect> boxes;
    std::vector<float> confidences;
    decode(scores, geometry, confThreshold, boxes, confidences);
    // Apply non-maximum suppression procedure.
    float confThreshold = 0.5;
    float nmsThreshold = 0.4;
    std::vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Render detections.
    Point2f ratio((float)frame.cols / inpWidth, (float)frame.rows / inpHeight);
    for (size_t i = 0; i < indices.size(); ++i){
        RotatedRect& box = boxes[indices[i]];
        Point2f vertices[4];
        box.points(vertices);
        for (int j = 0; j < 4; ++j){
            vertices[j].x *= ratio.x;
            vertices[j].y *= ratio.y;
        }
        for (int j = 0; j < 4; ++j)
            line(frame, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0), 1);
    }

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = format("Inference time: %.2f ms", t);
    putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
    imshow(kWinName, frame);
    return;
}

void vid_callback(const sensor_msgs::ImageConstPtr& msg){
    NodeHandle n;
    Publisher detec_text = n.advertise<std_msgs::String>("detection_result", 1000);
    std_msgs::String msg1;
    det_text = detection_east(cv_bridge::toCvShare(msg, "bgr8")->image);
    msg1.data = det_text;
    detec_text.publish(msg1);
    // stringstream ss;
    // ss<<center_x;
    // msg.data = ss.str();
    // detec_text.publish(msg);
    // ss<<center_y;
    // msg.data = ss.str();
    // detec_text.publish(msg);
    imshow("window", cv_bridge::toCvShare(msg, "bgr8")->image);
    waitKey(100);
}

int main(int argc, char **argv){
    init(argc, argv, "detector");
    NodeHandle n;
    namedWindow("window", WINDOW_NORMAL);
    startWindowThread();
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("videofeed", 1, vid_callback);
    spin();
    return 0;
}