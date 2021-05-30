#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>

std::vector<float> cur_pos(3,0);
std::vector<float> old_pos(3,0);
int old_id=1000;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
  
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  cur_pos[0]=msg->pose.pose.position.x;//x
  cur_pos[1]=msg->pose.pose.position.y;//y
  cur_pos[2]=yaw;//yaw

  //first time in callback
  if(old_pos[0]==0 && old_pos[1]==0 && old_pos[2]==0){
    old_pos=cur_pos;
    old_id=1000;
    ROS_INFO("VERTEX_SE2 %d %f %f %f",old_id, cur_pos[0],cur_pos[1],cur_pos[2]);
  }
  else{
    float distance_position= sqrt(pow(cur_pos[0]-old_pos[0],2)+pow(cur_pos[1]-old_pos[1],2));
    float distance_orientation= cur_pos[2]-old_pos[2];
    if(distance_position >= 0.1 || distance_orientation >= 0.5){
      old_id++;
      ROS_INFO("VERTEX_SE2 %d %f %f %f",old_id, cur_pos[0],cur_pos[1],cur_pos[2]);
      ROS_INFO("EDGE_SE2 %d %d %f %f %f",old_id-1, old_id, cur_pos[0]-old_pos[0],cur_pos[1]-old_pos[1],cur_pos[2]-old_pos[2]);
      old_pos=cur_pos;
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gen_graph_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("odom", 1000, odometryCallback_);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (n.ok()){
    ros::spinOnce();
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ROS_INFO("Prova: [%f,%f]",transformStamped.transform.translation.x,transformStamped.transform.translation.y);
    rate.sleep();
  }
  return 0;
};