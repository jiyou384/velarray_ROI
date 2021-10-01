#include <lidar/vlp16_subscriber.h>

namespace VlpSubscriber
{
void PointSubscribe::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{  
  ROS_WARN("callback");
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *lidar_cloud_);
  setROI(lidar_cloud_);
}

void PointSubscribe::setROI(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_) 
{
  ROS_WARN("ROI");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(lidar_cloud_);                
  pass.setFilterFieldName("z");            
  pass.setFilterLimits(0, 0.5); 
  // pass.setFilterFieldName("x");
  // pass.setFilterLimits(-1, 1);     
  pass.filter(*cloud_filtered); 
  publish(cloud_filtered);           
}

void PointSubscribe::subscribe()
{
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/hub0/v8_pcl", 1, &PointSubscribe::cloud_cb, this);
}

void PointSubscribe::publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2pub)
{
  ROS_WARN("pub");
  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  pub.publish(cloud2pub);
}
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_subscriber");

  VlpSubscriber::PointSubscribe pointsubscribe;
  pointsubscribe.subscribe();

  // Spin
  ros::spin ();
}
