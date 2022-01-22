#include "lidar_roi_example/vlp16_roi.h"

namespace ROIExample
{
  VLP16ROI::VLP16ROI()
  {
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &VLP16ROI::CloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("roi_cloud", 1);
  }

  void VLP16ROI::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *lidar_cloud_);
    SetROI(lidar_cloud_);
  }

  void VLP16ROI::SetROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1, 0);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2, 2);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 5);
    pass.filter(*cloud_filtered);

    pub_.publish(cloud_filtered);
  }
}
