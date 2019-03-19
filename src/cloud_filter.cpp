#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <realsense_postprocess/cloud_operator.h>


class CloudFilter : public CloudOperator
{
public:
    CloudFilter(ros::NodeHandle &nh) :
        cloud_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1))
    {}

    void operate()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;

        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(cloud_input_pcl.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered_pcl);

        pcl::toROSMsg(cloud_filtered_pcl, cloud_filterd_ros_);
    }

    void publish()
    {
        cloud_pub_.publish(cloud_filterd_ros_);
    }

protected:
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_filterd_ros_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_postprocess_cloud_filter");
    ros::NodeHandle nh;

    CloudOperationHandler handler(nh, new CloudFilter(nh), "/camera/depth/color/points");

    ros::spin();
    return 0;
}
