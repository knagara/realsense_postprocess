#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageFilter
{
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher image_publisher_;

public:
  ImageFilter()
  {
    ros::NodeHandle nh;
    image_transport::ImageTransport image_transport(nh);
    image_subscriber_ = image_transport.subscribe("/camera/color/image_raw", 1, &ImageFilter::imageCallback, this);
    image_publisher_ = image_transport.advertise("/image_filtered", 1);
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // ros msg -> cv Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    auto image_edge_8u = applySobelFilter(cv_ptr->image);

    // cv Mat -> ros msg
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_edge_8u).toImageMsg();
    
    image_publisher_.publish(output_msg);
  }

  cv::Mat applySobelFilter(const cv::Mat &input)
  {
    cv::Mat gray;
    cv::cvtColor(input, gray, CV_BGR2GRAY);
    cv::Mat edge_image;
    cv::Sobel(gray, edge_image, CV_32F, 1, 1, 3);
    cv::Mat image_edge_8u;
    edge_image.convertTo(image_edge_8u, CV_8U);
    return image_edge_8u;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense_postprocess_image_filter");

  ImageFilter imageFilter;

  ros::spin();
  return 0;
}