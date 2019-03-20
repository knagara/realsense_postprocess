#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>

#include <realsense_postprocess/cloud_operator.h>

class CloudFilter : public CloudOperator
{
public:
  CloudFilter(ros::NodeHandle &nh) : cloud_pub_(nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1))
  {
  }

  void operate()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;

    pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl);

    applyDownSampler(cloud_input_pcl, &cloud_filtered_pcl);

    pcl::toROSMsg(cloud_filtered_pcl, cloud_filterd_ros_);
  }

  void publish()
  {
    cloud_pub_.publish(cloud_filterd_ros_);
  }

private:
  void applyDownSampler(const pcl::PointCloud<pcl::PointXYZ> &input, pcl::PointCloud<pcl::PointXYZ> *output)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(input.makeShared());
    voxelSampler.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelSampler.filter(*output);
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
