#include <lidar/vlp16_subscriber.h>

namespace VlpSubscriber
{
void PointSubscribe::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{  
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
  pass.setFilterLimits(-1, 0); 
  pass.filter(*cloud_filtered); 

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1, 1);     
  pass.filter(*cloud_filtered); 

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0, 4);     
  pass.filter(*cloud_filtered); 

  int size = cloud_filtered->points.size();
  ROS_WARN("size %d", size);
  centroid(size, cloud_filtered);
  publish(cloud_filtered);           
}

void PointSubscribe::centroid(int size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  for(int i=0;i<size;i++){
    x += cloud_filtered->points[i].x;
    y += cloud_filtered->points[i].y;
    z += cloud_filtered->points[i].z;
  }
  float cen_x = x/size;
  float cen_y = y/size;
  float cen_z = z/size;
  visualize(cen_x, cen_y, cen_z);
}

void PointSubscribe::visualize(float x, float y, float z)
{
  pub2 = nh.advertise<visualization_msgs::Marker>("centroid", 1);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "lidar";
  marker.header.stamp = ros::Time();
  // marker.ns = "centroid";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  pub2.publish(marker);
  ROS_WARN("x %f y %f z %f", x, y, z);
}

void PointSubscribe::subscribe()
{
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/hub0/v8_pcl", 1, &PointSubscribe::cloud_cb, this);
}

void PointSubscribe::publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2pub)
{
  ROS_WARN("pub");
  pub = nh.advertise<sensor_msgs::PointCloud2>("roi_cloud", 1);
  pub.publish(cloud2pub);

  // geometry_msgs::Point point;
  // point.x = 1;
  // point.y = 1;
  // point.z = 1;
  // pub2 = nh.advertise<geometry_msgs::Point>("test_point", 1);
  // pub2.publish(point);
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
