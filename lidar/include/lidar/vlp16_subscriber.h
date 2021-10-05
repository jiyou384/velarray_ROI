#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>

namespace VlpSubscriber 
{
class PointSubscribe 
{
private:

public:
    //PointSubscribe();
    //~PointSubscribe();

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    void subscribe();
    void publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2pub);
    void setROI(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_ptr);
    double theta(const double &x, const double &y);
    void centroid(int size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
    void visualize(float x,float y, float z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;

    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::NodeHandle nh;
};
}