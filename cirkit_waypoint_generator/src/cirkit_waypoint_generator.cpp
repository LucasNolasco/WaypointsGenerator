#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <cirkit_waypoint_manager_msgs/WaypointArray.h>

#include <sensor_msgs/NavSatFix.h>

#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include <boost/tokenizer.hpp>
#include <boost/shared_array.hpp>
#include <boost/program_options.hpp>

using namespace visualization_msgs;


typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server; /* Interactive marker for RViz */

class CirkitWaypointGenerator
{
public:
  CirkitWaypointGenerator():
    rate_(5)
  {
    ros::NodeHandle n("~");
    n.param("dist_th", dist_th_, 1.0); // distance threshold [m]
    n.param("yaw_th", yaw_th_, 45.0*3.1415/180.0); // yaw threshold [rad]

    std::cout << "PARAMETER DIST_TH: " << dist_th_ << std::endl;

    // odom_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",
    //                           1,
    //                           &CirkitWaypointGenerator::addWaypoint, this);
    gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/fix",
                              1,
                              &CirkitWaypointGenerator::addWaypoint, this);
    // clicked_sub_ = nh_.subscribe("clicked_point", 1, &CirkitWaypointGenerator::clickedPointCallback, this);
    reach_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/reach_threshold_markers", 1);
    waypoints_pub_ = nh_.advertise<cirkit_waypoint_manager_msgs::WaypointArray>("/waypoints", 1);
    waypoint_box_count_ = 0;
    server.reset( new interactive_markers::InteractiveMarkerServer("cube") );
  }

  void load(std::string waypoint_file)
  {
    const int rows_num = 25; // id, x, y, z, Qx, Qy, Qz, Qw, status, status_service, latitude, longitude, altitude, covariance (9), covariance_type, is_searching_area, reach_threshold
    boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
    std::ifstream ifs(waypoint_file.c_str());
    std::string line;
    while(ifs.good()){
      getline(ifs, line);
      if(line.empty()){ break; }
      tokenizer tokens(line, sep);
      std::vector<double> data;
      tokenizer::iterator it = tokens.begin();
      for(; it != tokens.end() ; ++it){
        std::stringstream ss;
        double d;
        ss << *it;
        ss >> d;
        data.push_back(d);
      }
      if(data.size() != rows_num){
        ROS_ERROR("Row size is mismatch!!");
        return;
      }else{
        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.position.x = data[1];
        new_pose.pose.position.y = data[2];
        new_pose.pose.position.z = data[3];
        new_pose.pose.orientation.x = data[4];
        new_pose.pose.orientation.y = data[5];
        new_pose.pose.orientation.z = data[6];
        new_pose.pose.orientation.w = data[7];

        sensor_msgs::NavSatFix new_fix;
        new_fix.status.status = data[8];
        new_fix.status.service = data[9];
        new_fix.latitude = data[10];
        new_fix.longitude = data[11];
        new_fix.altitude = data[12];

        for(int i = 0; i < 9; i++) {
          new_fix.position_covariance[i] = data[13 + i];
        }

        new_fix.position_covariance_type = data[22];

        makeWaypointMarker(new_pose, new_fix, (int)data[23], data[24]);
      }
    }
    ROS_INFO_STREAM(waypoint_box_count_ << "waypoints are loaded.");
  }

  double calculateDistance(geometry_msgs::PoseStamped new_pose)
  {
    return sqrt(pow(new_pose.pose.position.x - last_pose_.pose.position.x, 2)
                + pow(new_pose.pose.position.y - last_pose_.pose.position.y, 2));
  }

  void getRPY(const geometry_msgs::Quaternion &q,
              double &roll,double &pitch,double &yaw){
    tf::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  }

  double calculateAngle(geometry_msgs::PoseStamped new_pose)
  {
    double yaw, pitch, roll;
    getRPY(new_pose.pose.orientation, roll, pitch, yaw);
    double last_yaw, last_pitch, last_roll;
    getRPY(last_pose_.pose.orientation, last_roll, last_pitch, last_yaw);
    return fabs(yaw - last_yaw);
  }

  InteractiveMarkerControl& makeWaypointMarkerControl(InteractiveMarker &msg,
                                                      int is_searching_area)
  {
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    msg.controls.push_back(control);
    // control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    // msg.controls.push_back(control);
    // control.independent_marker_orientation = true;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    msg.controls.push_back(control);

    Marker marker;
    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale*0.5;
    marker.scale.y = msg.scale*0.5;
    marker.scale.z = msg.scale*0.5;
    marker.color.r = 0.05 + 1.0*(float)is_searching_area;
    marker.color.g = 0.80;
    marker.color.b = 0.02;
    marker.color.a = 1.0;

    control.markers.push_back(marker);
    control.always_visible = true;
    msg.controls.push_back(control);

    return msg.controls.back();

  }

  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";
  
    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        {
          reach_threshold_markers_.markers[std::stoi(feedback->marker_name)].pose
            = feedback->pose;
          reach_marker_pub_.publish(reach_threshold_markers_);
          waypoints_.waypoints[std::stoi(feedback->marker_name)].pose = feedback->pose;
          break;
        }
    }
    server->applyChanges();
  }
  
  void makeWaypointMarker(const geometry_msgs::PoseStamped new_pose, const sensor_msgs::NavSatFix gps_fix, int is_searching_area, double reach_threshold)
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.pose = new_pose.pose;
    int_marker.scale = 1;

    visualization_msgs::Marker reach_marker;
    reach_marker.header.frame_id = "map";
    reach_marker.header.stamp = ros::Time();
    reach_marker.id = waypoint_box_count_;
    reach_marker.type = visualization_msgs::Marker::CYLINDER;
    reach_marker.action = visualization_msgs::Marker::ADD;
    reach_marker.pose = new_pose.pose;
    reach_marker.scale.x = reach_threshold/2.0;
    reach_marker.scale.y = reach_threshold/2.0;
    reach_marker.scale.z = 0.1;
    reach_marker.color.a = 0.7;
    reach_marker.color.r = 0.0;
    reach_marker.color.g = 0.0;
    reach_marker.color.b = 1.0;
    reach_threshold_markers_.markers.push_back(reach_marker);
    reach_marker_pub_.publish(reach_threshold_markers_);
    
    std::stringstream s;
    s << waypoint_box_count_;
    int_marker.name = s.str();
    int_marker.description = s.str();

    makeWaypointMarkerControl(int_marker, is_searching_area);

    server->insert(int_marker);
    server->setCallback(int_marker.name, boost::bind(&CirkitWaypointGenerator::processFeedback, this, _1));
    server->applyChanges();

    cirkit_waypoint_manager_msgs::Waypoint waypoint;
    waypoint.id = waypoint_box_count_;
    waypoint.pose = new_pose.pose;
    waypoint.gps_fix = gps_fix;
    waypoint.is_search_area = is_searching_area;
    waypoint.reach_tolerance = reach_threshold/2.0;
    waypoints_.waypoints.push_back(waypoint);
    
    waypoint_box_count_++;
  }

  bool getPosition(geometry_msgs::PoseStamped* pose_stamped) {
    bool success = false;
    tf::StampedTransform transform;
    try {
      geometry_msgs::PoseStamped base_link_origin;
      base_link_origin.header.stamp = ros::Time(0);
      base_link_origin.header.frame_id = "base_link";
      base_link_origin.pose.position.x = 0;
      base_link_origin.pose.position.y = 0;
      base_link_origin.pose.position.z = 0;

      base_link_origin.pose.orientation.x = 0;
      base_link_origin.pose.orientation.y = 0;
      base_link_origin.pose.orientation.z = 0;
      base_link_origin.pose.orientation.w = 1;

      const std::string target_frame = "map";

      listener.transformPose(target_frame, base_link_origin, *pose_stamped);

      success = true;
    } catch(tf::TransformException ex) {
       ROS_INFO("Error on TF");
    }

    return success;
  }

  void addWaypoint(const sensor_msgs::NavSatFix::ConstPtr& fix) // (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
  {
    geometry_msgs::PoseStamped map_pose;

    ROS_INFO_STREAM("GPS Fix received");
    
    if(getPosition(&map_pose)) {
      ROS_INFO_STREAM("Map Pose received");
      double diff_dist = calculateDistance(map_pose);
      double diff_yaw = calculateAngle(map_pose);
      if(diff_dist > dist_th_ || diff_yaw > yaw_th_)
      {
        ROS_INFO("Adding new waypoint");
        makeWaypointMarker(map_pose, *fix, 0, 3.0);
        last_pose_ = map_pose;
      }
    }

    // double diff_dist = calculateDistance(amcl_pose->pose);
    // double diff_yaw = calculateAngle(amcl_pose->pose);
    // if(diff_dist > dist_th_ || diff_yaw > yaw_th_)
    // {
    //   makeWaypointMarker(amcl_pose->pose, 0, 3.0);
    //   last_pose_ = amcl_pose->pose;
    // }
  }

  void publishWaypointCallback(const ros::TimerEvent&)
  {
    reach_marker_pub_.publish(reach_threshold_markers_);
    waypoints_pub_.publish(waypoints_);
    server->applyChanges();
  }

  // void clickedPointCallback(const geometry_msgs::PointStamped &point)
  // {
  //   geometry_msgs::PoseStamped pose;
  //   tf::pointTFToMsg(tf::Vector3( point.point.x, point.point.y, 0), pose.pose.position);
  //   tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0), pose.pose.orientation);
  //   makeWaypointMarker(pose, 0, 3.0);
  //   server->applyChanges();
  // }
  
  void tfSendTransformCallback(const ros::TimerEvent&)
  {  
    tf::Transform t;
    ros::Time time = ros::Time::now();

    for (size_t i = 0; i < waypoints_.waypoints.size(); ++i) {
      std::stringstream s;
      s << waypoints_.waypoints[i].id;
      t.setOrigin(tf::Vector3(waypoints_.waypoints[i].pose.position.x,
                              waypoints_.waypoints[i].pose.position.y,
                              waypoints_.waypoints[i].pose.position.z));
      t.setRotation(tf::Quaternion(waypoints_.waypoints[i].pose.orientation.x,
                                   waypoints_.waypoints[i].pose.orientation.y,
                                   waypoints_.waypoints[i].pose.orientation.z,
                                   waypoints_.waypoints[i].pose.orientation.w));
      br_.sendTransform(tf::StampedTransform(t, time, "map", s.str()));
    }
  }
  
  void run()
  {
    ros::Timer frame_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&CirkitWaypointGenerator::publishWaypointCallback, this, _1));
    ros::Timer tf_frame_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&CirkitWaypointGenerator::tfSendTransformCallback, this, _1));
    while(ros::ok())
    {
      ros::spinOnce();
      rate_.sleep();
    }
  }
private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  //ros::Subscriber odom_sub_;
  ros::Subscriber gps_sub_;
  // ros::Subscriber clicked_sub_;
  ros::Publisher reach_marker_pub_;
  ros::Publisher waypoints_pub_;
  geometry_msgs::PoseStamped last_pose_;
  cirkit_waypoint_manager_msgs::WaypointArray waypoints_;
  double dist_th_;
  double yaw_th_;
  int waypoint_box_count_;
  std::vector<double> reach_thresholds_;
  visualization_msgs::MarkerArray reach_threshold_markers_;
  tf::TransformBroadcaster br_;
  tf::TransformListener listener;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_generator");
  CirkitWaypointGenerator generator;

  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message")
    ("load", boost::program_options::value<std::string>(), "waypoint filename");

  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    if( vm.count("help") ){
      std::cout << "This is waypoint generator node" << std::endl;
      std::cerr << desc << std::endl;
      return 0;
    }
    if (vm.count("load")) {
      generator.load(vm["load"].as<std::string>());
    }
  } catch (boost::program_options::error& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return -1;
  }

  generator.run();
  return 0;
}
