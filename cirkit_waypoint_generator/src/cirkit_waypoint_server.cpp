#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

using namespace visualization_msgs;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

class CirkitWaypointServer {
public:
    CirkitWaypointServer() : rate_(5) {
        ros::NodeHandle n("~");
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_markers", 1);
    }

    void load(std::string waypoint_file) {
        const int rows_num = 25; // id, x, y, z, Qx, Qy, Qz, Qw, status, status_service, latitude, longitude, altitude, covariance (9), covariance_type, is_searching_area, reach_threshold
        boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
        std::ifstream ifs(waypoint_file.c_str());
        std::string line;

        waypoint_box_count_ = 0;

        while(ifs.good()) {
            getline(ifs, line);
            if(line.empty()) {
                break; 
            }

            tokenizer tokens(line, sep);
            std::vector<double> data;
            tokenizer::iterator it = tokens.begin();
            
            for(; it != tokens.end() ; ++it) {
                std::stringstream ss;
                double d;
                ss << *it;
                ss >> d;
                data.push_back(d);
            }
            
            if(data.size() != rows_num) {
                ROS_ERROR("Row size is mismatch!!");
                return;
            }
            else {
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

                makeWaypointMarker(new_pose, (int)data[23], data[24]);
            }
        }
        
        ROS_INFO_STREAM(waypoint_box_count_ << " waypoints are loaded.");
    }

    void getRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw) {
        tf::Quaternion tfq(q.x, q.y, q.z, q.w);
        tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
    }

    void makeWaypointMarker(const geometry_msgs::PoseStamped new_pose, int is_searching_area, double reach_threshold) {
        visualization_msgs::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "map";
        waypoint_marker.header.stamp = ros::Time();
        waypoint_marker.id = waypoint_box_count_;
        waypoint_marker.type = visualization_msgs::Marker::ARROW;
        waypoint_marker.action = visualization_msgs::Marker::ADD;
        waypoint_marker.pose = new_pose.pose;
        waypoint_marker.scale.x = 0.8;
        waypoint_marker.scale.y = 0.5;
        waypoint_marker.scale.z = 0.0;
        waypoint_marker.color.a = 0.7;
        waypoint_marker.color.r = 0.05 + 1.0*(float)is_searching_area;;
        waypoint_marker.color.g = 0.80;
        waypoint_marker.color.b = 0.2;
        waypoint_arrow_markers_.markers.push_back(waypoint_marker);
        waypoint_marker_pub_.publish(waypoint_arrow_markers_);
        waypoint_box_count_++;
    }

    void publishWaypointCallback(const ros::TimerEvent&) {
        waypoint_marker_pub_.publish(waypoint_arrow_markers_);
    }

    void run() {
        ros::Timer frame_timer = nh_.createTimer(ros::Duration(0.1), boost::bind(&CirkitWaypointServer::publishWaypointCallback, this, _1));
   
        while(ros::ok()) {
            ros::spinOnce();
            rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Rate rate_;

    ros::Publisher waypoint_marker_pub_;
    int waypoint_box_count_;
    visualization_msgs::MarkerArray waypoint_arrow_markers_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_server");
    CirkitWaypointServer waypoint_server;

    boost::program_options::options_description desc("Options");
    desc.add_options()
        ("help", "Print help message")
        ("load", boost::program_options::value<std::string>(), "waypoint filename");

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        if(vm.count("help")) {
            std::cout << "This is waypoint generator node" << std::endl;
            std::cerr << desc << std::endl;
            return 0;
        }
        if(vm.count("load")) {
            waypoint_server.load(vm["load"].as<std::string>());
        }
    } 
    catch (boost::program_options::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return -1;
    }

    waypoint_server.run();
    return 0;
}
