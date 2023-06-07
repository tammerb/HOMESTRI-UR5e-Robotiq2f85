#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <mutex>


sensor_msgs::PointCloud2 pc_msg;
std::mutex m;
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  m.lock();
  pc_msg = *input;
  m.unlock();
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();
  

  tf::TransformListener listener;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloud_cb);
  
  pcl::PointCloud<pcl::PointXYZ> merged_pc;
  for (int i = 0; i < 3; i++ ) {
    ROS_INFO("PRESS ENTER TO TAKE PICTURE");

    getchar();

    m.lock();
    sensor_msgs::PointCloud2 cur_pc_msg = pc_msg;
    m.unlock();

    sensor_msgs::PointCloud2 world_pc_msg;
    pcl_ros::transformPointCloud("world", cur_pc_msg, world_pc_msg, listener);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(world_pc_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromPCLPointCloud2(pcl_pc2, pc);

    merged_pc += pc;
  }

  pcl::io::savePCDFileBinary("point_cloud.pcd", merged_pc);
}