#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/MarkerArray.h>
#include <cirkit_waypoint_manager_msgs/WaypointArray.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>

#include <boost/date_time.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

// bool compareInteractiveMarker(visualization_msgs::InteractiveMarker left, visualization_msgs::InteractiveMarker right) {
//     return std::stoi(left.name) < std::stoi(right.name);
// }

/*------------------------------------------------------------------------------
 *  Name: timeToStr
 *      Creates a string using the current time.
 *
 *  Return description: String on the format YYYY-MM-DD-HH-MM-SS
 *
 *---------------------------------------------------------------------------*/
std::string timeToStr() {
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

class CirkitWaypointSaver {
public:
    /*------------------------------------------------------------------------------
    *  Name: CirkitWaypointSaver
    *      Subscribe to "/waypoints" topic.
    *
    *      waypoints_file - Path to file where waypoints data will be stored
    *
    *---------------------------------------------------------------------------*/
    CirkitWaypointSaver(const std::string& waypoints_file) : waypoints_file_(waypoints_file), saved_waypoints_(false) {
        ros::NodeHandle n;
    
        waypoints_sub_ = n.subscribe("/waypoints", 1, &CirkitWaypointSaver::waypointsCallback, this);
        ROS_INFO("Waiting for waypoints");
    }

    /*------------------------------------------------------------------------------
    *  Name: waypointsCallback
    *      Callback to "/waypoints" topic. Store waypoints data on file using the following format:
    *          id, x, y, z, Qx, Qy, Qz, Qw, status, status_service, latitude, longitude, altitude, covariance (9), covariance_type, is_searching_area, reach_threshold
    *
    *      waypoints - Waypoint array received from "/waypoints"
    *
    *---------------------------------------------------------------------------*/
    void waypointsCallback(cirkit_waypoint_manager_msgs::WaypointArray waypoints) {
        ROS_INFO("Received waypoints : %d", (int)waypoints.waypoints.size());

        std::ofstream savefile(waypoints_file_.c_str(), std::ios::out);
    
        size_t size = waypoints.waypoints.size();
        for(unsigned int i = 0; i < size; i++) {
            savefile << waypoints.waypoints[i].id << ","  /* Waypoint ID */
               
                     /* Stamped Position */
                     << waypoints.waypoints[i].pose.position.x << ","
                     << waypoints.waypoints[i].pose.position.y << ","
                     << waypoints.waypoints[i].pose.position.z << "," 
                     << waypoints.waypoints[i].pose.orientation.x << ","
                     << waypoints.waypoints[i].pose.orientation.y << ","
                     << waypoints.waypoints[i].pose.orientation.z << ","
                     << waypoints.waypoints[i].pose.orientation.w << ","

                     /* GPS Fix */
                     << static_cast<uint16_t> (waypoints.waypoints[i].gps_fix.status.status) << ","
                     << waypoints.waypoints[i].gps_fix.status.service << ","
                     << waypoints.waypoints[i].gps_fix.latitude << ","
                     << waypoints.waypoints[i].gps_fix.longitude << ","
                     << waypoints.waypoints[i].gps_fix.altitude << ",";

                     for(int j = 0; j < 9; j++) {
                        savefile << waypoints.waypoints[i].gps_fix.position_covariance[j] << ",";
                     }

                     savefile << static_cast<uint16_t> (waypoints.waypoints[i].gps_fix.position_covariance_type) << ","

                     << waypoints.waypoints[i].is_inspection_area << ","
                     << waypoints.waypoints[i].reach_tolerance * 2.0 << std::endl;
      
            ROS_INFO_STREAM("ID: " << waypoints.waypoints[i].id << " Fix Status: " << static_cast<uint16_t> (waypoints.waypoints[i].gps_fix.status.status) << " Cov Type: " << static_cast<uint16_t> (waypoints.waypoints[i].gps_fix.position_covariance_type));
        }

        saved_waypoints_ = true;
        ROS_INFO_STREAM("Saved to : " << waypoints_file_);
    }
  
    std::string waypoints_file_; /* Path to the file where the data will be stored */
    ros::Subscriber waypoints_sub_; /* Subscriber to "/waypoints" */
    bool saved_waypoints_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_saver");
    std::string waypoints_name = timeToStr() + ".csv";
    
    CirkitWaypointSaver saver(waypoints_name);
    while(!saver.saved_waypoints_ && ros::ok()) {
        ros::spinOnce();
    }
    
    return 0;
}
