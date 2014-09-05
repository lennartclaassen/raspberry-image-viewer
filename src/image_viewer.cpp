#include <raspberry_image_viewer/image_viewer.h>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

imageViewer::imageViewer() {

    this->nh        = new ros::NodeHandle;
    this->it        = new image_transport::ImageTransport(*this->nh);

    nh->param<int>("/raspberry_image_viewer/display_width", this->displayResolution.width, 848);
    nh->param<int>("/raspberry_image_viewer/display_height", this->displayResolution.height, 480);
    nh->param<std::string>("/raspberry_image_viewer/image_topic", this->topicName, "/raspberry_image");

    this->image_sub = this->it->subscribe(this->topicName, 5, &imageViewer::imageCallback, this);

    this->imageReceived = false;
    this->windowName = "display";
    this->waitScreen = Mat::ones(this->displayResolution, CV_8U);

    this->setupDisplay();

    ROS_INFO("initialized");
}

void imageViewer::setupDisplay() {
        namedWindow(this->windowName, WINDOW_NORMAL);
        resizeWindow(this->windowName, this->displayResolution.width, this->displayResolution.height);
        setWindowProperty(this->windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        imshow(this->windowName, this->waitScreen);
        waitKey(10);
        ROS_INFO("setup complete");
}

void imageViewer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        this->cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        this->receivedFrame = cv_ptr->image;
        this->imageReceived = true;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void imageViewer::showImage(Mat &img) {
    imshow(this->windowName, this->receivedFrame);
    waitKey(1);
}

void imageViewer::loop() {
    while(ros::ok() && !this->imageReceived){
        ROS_ERROR("No image received yet ...");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    ROS_INFO("run");

    while(ros::ok()) {
        this->showImage(this->receivedFrame);
        ros::spinOnce();
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "raspberry_image_viewer");
    imageViewer viewer;

    viewer.loop();
}
