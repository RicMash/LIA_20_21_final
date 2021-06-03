#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <fstream>
#include <set>
#include <math.h>

std::set<int> detected_tags;
std::vector<float> cur_pos(3,0);
//not instantiated so we know the first time must be equal to cur_pos
std::vector<float> old_pos;
int old_id=1000;
std::ofstream output;
geometry_msgs::TransformStamped transformStamped;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  cur_pos[0]=msg->pose.pose.position.x;//x
  cur_pos[1]=msg->pose.pose.position.y;//y
  cur_pos[2]=yaw;//yaw

  //first time in callback
  if(old_pos.empty()){
    old_pos=cur_pos;
    output << "VERTEX_SE2 " << old_id << " " << cur_pos[0] << " " << cur_pos[1] << " " << cur_pos[2] << "\n";
  }
  else{
    float distance_position= sqrt(pow(cur_pos[0]-old_pos[0],2)+pow(cur_pos[1]-old_pos[1],2));
    float distance_orientation= cur_pos[2]-old_pos[2];
    if(distance_position >= 0.1 || distance_orientation >= 0.5){
      old_id++;
      output << "VERTEX_SE2 " << old_id << " " << cur_pos[0] << " " << cur_pos[1] << " " << cur_pos[2] << "\n";
      output << "EDGE_SE2 " << old_id-1 << " " << old_id << " " << cur_pos[0]-old_pos[0] << " " << cur_pos[1]-old_pos[1] << " " << cur_pos[2]-old_pos[2] << "\n";
      old_pos=cur_pos;
    }
  }
}

void tagDetectedCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr msg) {
  for(int i=0;i<msg->detections.size();i++){
    for(int j=0;j<msg->detections[i].id.size();j++){  
      int current_tag_id = msg->detections[i].id[j];

      //must take the position with reference to the old position, not the current, because in output we won't have the current position, just the old one
      tf2::Vector3 tag_current_view_p = tf2::Vector3(msg->detections[i].pose.pose.pose.position.x, msg->detections[i].pose.pose.pose.position.y, msg->detections[i].pose.pose.pose.position.z);
      tf2::Vector3 translation = tf2::Vector3(cur_pos[0]-old_pos[0], cur_pos[1]-old_pos[1], 0);
      tf2::Quaternion rotation; 
      rotation.setRPY(0,0,cur_pos[2]-old_pos[2]);
      tf2::Transform tf_offset = tf2::Transform(rotation, translation);
      
      tf2::Vector3 tag_old_view_p = tf_offset * tag_current_view_p;
      
      output << "EDGE_SE2_XY " << old_id << " " << current_tag_id << " " <<  tag_old_view_p.x() << " " << tag_old_view_p.y() << "\n";
      
      if(detected_tags.find(current_tag_id)==detected_tags.end()){
        detected_tags.insert(current_tag_id);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        try{          
          geometry_msgs::PointStamped tag_current_view_ps, tag_world_view_ps;
          
          //create the PointStamped that has to be transformed
          tag_current_view_ps.header.frame_id="fisheye_rect";
          tag_current_view_ps.header.stamp=ros::Time(0);
          tag_current_view_ps.point.x=msg->detections[i].pose.pose.pose.position.x;
          tag_current_view_ps.point.y=msg->detections[i].pose.pose.pose.position.y;
          tag_current_view_ps.point.z=0;

          //transform the point
          tag_world_view_ps= tfBuffer.transform(tag_current_view_ps,"odom",ros::Duration(3.0));
          
          output << "VERTEX_XY " << current_tag_id << " " << tag_world_view_ps.point.x << " " << tag_world_view_ps.point.y << "\n";
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
        }
      }
    }
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "gen_graph_node");

  ros::NodeHandle n;
  output.open("result.g2o");
  ros::Subscriber sub_tag = n.subscribe("tag_detections", 1000, tagDetectedCallback);
  ros::Subscriber sub = n.subscribe("odom", 1000, odometryCallback_);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while (n.ok()){
    ros::spinOnce();
  }
  output.close();
  return 0;
};