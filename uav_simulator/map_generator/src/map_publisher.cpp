// Nap ban do .pcd va phat len /map_generator/global_cloud,
// thay cho node random_forest khi chay ban do Indoor.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle n("~");

  std::string map_file, frame_id;
  double rate;
  n.param<std::string>("map_file", map_file, std::string(""));
  n.param<std::string>("frame_id", frame_id, std::string("world"));
  n.param("rate", rate, 10.0);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (map_file.empty() || pcl::io::loadPCDFile(map_file, cloud) == -1) {
    ROS_FATAL("[map_publisher] Khong nap duoc ban do: '%s'", map_file.c_str());
    return 1;
  }
  ROS_WARN("[map_publisher] Da nap %s - %lu diem",
           map_file.c_str(), (unsigned long)cloud.points.size());

  // Chuyen doi MOT LAN, roi phat lai cung mot ban tin.
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;

  ros::Publisher pub =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  ros::Rate loop(rate);
  while (ros::ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}