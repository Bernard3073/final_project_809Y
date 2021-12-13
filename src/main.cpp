#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <array>
#include <utility>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<int> t;
std::vector<std::vector<int>> target(4, t);

void broadcast() {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}

void listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  int i;
  if(ros::param::has("target_num")){
    ros::param::get("target_num", i);
    target[i].push_back(transformStamped.transform.translation.x);
    target[i].push_back(transformStamped.transform.translation.y);
  }
  
}


void marker_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& marker){
  // marker->transforms.empty
  std::cout << marker->transforms.empty() << '\n';
  if(marker->transforms.empty()){
    ROS_INFO("CANNOT FIND MARKER !!");
    ros::param::set("find_marker", false);
  }
  else{
    ROS_INFO("FIND MARKER !!");
    ros::param::set("find_marker", true);
    
    //broadcaster object
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    transformStamped.transform.translation.x = marker->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = marker->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = marker->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = marker->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = marker->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = marker->transforms[0].transform.rotation.z;
    ros::param::set("target_num", marker->transforms[0].fiducial_id);
  }

  
}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  XmlRpc::XmlRpcValue loc_list;
  std::vector<double> d;
  std::vector<std::vector<double>> target_list(4, d);
  nh.getParam("/aruco_lookup_locations/target_1", loc_list);
  for(int i=0; i<loc_list.size(); i++){
    target_list[0].push_back(static_cast<double>(loc_list[i]));
  }
  nh.getParam("/aruco_lookup_locations/target_2", loc_list);
  for(int i=0; i<loc_list.size(); i++){
    target_list[1].push_back(static_cast<double>(loc_list[i]));
  }
  nh.getParam("/aruco_lookup_locations/target_3", loc_list);
  for(int i=0; i<loc_list.size(); i++){
    target_list[2].push_back(static_cast<double>(loc_list[i]));
  }
  nh.getParam("/aruco_lookup_locations/target_4", loc_list);
  for(int i=0; i<loc_list.size(); i++){
    target_list[3].push_back(static_cast<double>(loc_list[i]));
  }
  
  // ROS_INFO("%s\n",loc_1.c_str());
  //Build goal for explorer
  // explorer_goal.target_pose.header.frame_id = "map";
  // explorer_goal.target_pose.header.stamp = ros::Time::now();
  // explorer_goal.target_pose.pose.position.x = target_1.first;//
  // explorer_goal.target_pose.pose.position.y = target_1.second;//
  // explorer_goal.target_pose.pose.orientation.w = 1.0;
  
  // //Build goal for follower
  // follower_goal.target_pose.header.frame_id = "map";
  // follower_goal.target_pose.header.stamp = ros::Time::now();
  // follower_goal.target_pose.pose.position.x = -0.289296;//
  // follower_goal.target_pose.pose.position.y = -1.282680;//
  // follower_goal.target_pose.pose.orientation.w = 1.0;


  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
  // explorer_client.waitForResult();

  ros::Rate loop_rate(10);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel",100);
  geometry_msgs::Twist msg;
  ros::Subscriber marker_sub = nh.subscribe("/fiducial_transforms", 1, &marker_callback);
  
  bool find_marker;
  // int marker_id = 0;
  ros::param::set("target_num", 0);
  int marker_id;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = target_list[0][0];
  explorer_goal.target_pose.pose.position.y = target_list[0][1];
  explorer_goal.target_pose.pose.orientation.w = 1.0;
  int next_target_num = 1;
  while (ros::ok()) {

    nh.getParam("find_marker", find_marker);
    // maker sure rosparam target_num exists
    if(ros::param::has("target_num")){
      ros::param::get("target_num", marker_id);
      
    }

    if (!explorer_goal_sent)     {
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goal);//this should be sent only once
      explorer_goal_sent = true;

      
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, robot reached goal");

      
      if(find_marker == true){
        msg.linear.x = 0;
        msg.angular.z = 0;
        pub.publish(msg);
        explorer_goal_sent = false;
        follower_goal_sent = false;
        explorer_goal.target_pose.header.frame_id = "map";
        explorer_goal.target_pose.header.stamp = ros::Time::now();
        if(next_target_num != 4){
          explorer_goal.target_pose.pose.position.x = target_list[next_target_num][0];//
          explorer_goal.target_pose.pose.position.y = target_list[next_target_num][1];//
          explorer_goal.target_pose.pose.orientation.w = 1.0;
          next_target_num++;
        }
        else{
          explorer_goal.target_pose.pose.position.x = -4;//
          explorer_goal.target_pose.pose.position.y = 2.5;//
          explorer_goal.target_pose.pose.orientation.w = 1.0;
        }
      }
      else{
        msg.linear.x = 0;
        msg.angular.z = 0.1;
      }
      pub.publish(msg);
      
    }
    if (!follower_goal_sent) {
      ROS_INFO("Sending goal for explorer");
      follower_client.sendGoal(follower_goal);//this should be sent only once
      follower_goal_sent = true;
    }
    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, robot reached goal");
    }
    broadcast();
    listen(tfBuffer);
    ros::spinOnce(); //uncomment this if you have subscribers in your code

    loop_rate.sleep();

  }

}

