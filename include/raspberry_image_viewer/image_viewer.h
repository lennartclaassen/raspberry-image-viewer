#ifndef RASPBERRY_IMAGE_VIEWER_H
#define RASPBERRY_IMAGE_VIEWER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>


/**
 * @class imageViewer
 * @brief The image_viewer class
 */
class imageViewer {

public:
    imageViewer();

    void setupDisplay();
    void loop();

private:
    ros::NodeHandle* nh;

    image_transport::Subscriber         image_sub;
    image_transport::ImageTransport*    it;
    cv::Mat                             receivedFrame;
    cv::Mat                             waitScreen;
    cv_bridge::CvImagePtr               cv_ptr;

    std::string     topicName;
    std::string     windowName;
    cv::Size        displayResolution;
    bool            imageReceived;

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void showImage(cv::Mat &img);

};

#endif
