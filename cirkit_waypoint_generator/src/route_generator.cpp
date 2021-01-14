#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <cirkit_waypoint_manager_msgs/WaypointArray.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/shared_array.hpp>
#include <boost/tokenizer.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

class RouteGenerator
{
public:
    /*------------------------------------------------------------------------------
    *  Name: RouteGenerator
    *      Load waypoints_path parameter.
    * 
    *---------------------------------------------------------------------------*/
    RouteGenerator() : rate_(5) {
        ros::NodeHandle n("~");

        std::string waypoints_path;
        if(n.getParam("waypoints_path", waypoints_path)) {
            load(waypoints_path);
        }
        else {
            ROS_ERROR("Waypoints file path not informed");
        }
    }

    /*------------------------------------------------------------------------------
    *  Name: load
    *      Load waypoints stored on csv. Each csv row must be on the following format:
    *         id, x, y, z, Qx, Qy, Qz, Qw, status, status_service, latitude, longitude, altitude, covariance (9), covariance_type, is_searching_area, reach_threshold
    *
    *      waypoint_file - Path for the csv containing waypoints data
    *---------------------------------------------------------------------------*/
    void load(std::string waypoint_file) {
        const int rows_num = 25;
        boost::char_separator<char> sep("," ,"", boost::keep_empty_tokens);
        std::ifstream ifs(waypoint_file.c_str());
        std::string line;

        std::cout << "File path: " << waypoint_file.c_str() << " " << ifs.good() << std::endl;

        while(ifs.good()) {
            getline(ifs, line);
            if(line.empty()) { 
                break; 
            }

            tokenizer tokens(line, sep);
            std::vector<double> data;
            tokenizer::iterator it = tokens.begin();
            
            /* Split the row */
            for(; it != tokens.end() ; ++it) {
                std::stringstream ss;
                double d;
                ss << *it;
                ss >> d;
                data.push_back(d);
            }
            
            /* Checks the number of columns on this row */
            if(data.size() != rows_num) {
                ROS_ERROR("Row size is mismatch!!");
                return;
            } 
            else { /* Rebuild a new waypoint */
                cirkit_waypoint_manager_msgs::Waypoint waypoint;
                waypoint.id = data[0];

                waypoint.pose.position.x = data[1];
                waypoint.pose.position.y = data[2];
                waypoint.pose.position.z = data[3];
                waypoint.pose.orientation.x = data[4];
                waypoint.pose.orientation.y = data[5];
                waypoint.pose.orientation.z = data[6];
                waypoint.pose.orientation.w = data[7];
                
                waypoint.gps_fix.status.status = data[8];
                waypoint.gps_fix.status.service = data[9];
                waypoint.gps_fix.latitude = data[10];
                waypoint.gps_fix.longitude = data[11];
                waypoint.gps_fix.altitude = data[12];

                for(int i = 0; i < 9; i++) {
                waypoint.gps_fix.position_covariance[i] = data[13 + i];
                }

                waypoint.gps_fix.position_covariance_type = data[22];

                waypoint.is_inspection_area = data[23];
                waypoint.reach_tolerance = data[24]/2.0;
                waypoints_.waypoints.push_back(waypoint);
            }
        }
        ROS_INFO_STREAM(waypoints_.waypoints.size() << " waypoints are loaded.");
    }

    /*------------------------------------------------------------------------------
    *  Name: buildRoute
    *      Creates a route building a new WaypointArray sorted using the given id order.
    *
    *      path             - Array with waypoints ids of the path
    *      waypoints_type   - Array containing activity for each of the waypoints on path array
    *                         0 - Just passing through and 1 - Inspection
    *      path_length      - Number of waypoints on path
    *
    *  Return description: WaypointArray containing the route to be executed
    *
    *---------------------------------------------------------------------------*/
    cirkit_waypoint_manager_msgs::WaypointArray buildRoute(int* path, int* waypoint_type, int path_length) {
        cirkit_waypoint_manager_msgs::WaypointArray route;

        for(int i = 0; i < path_length; i++) {
            bool waypoint_found = false;
            for(int j = 0; j < waypoints_.waypoints.size() && !waypoint_found; j++) {
                if(path[i] == waypoints_.waypoints[j].id) {
                    cirkit_waypoint_manager_msgs::Waypoint step = waypoints_.waypoints[j];
                    step.is_inspection_area = waypoint_type[i];
                    route.waypoints.push_back(step);

                    waypoint_found = true;
                }
            }
            
            if(!waypoint_found) {
                ROS_INFO("Waypoint with ID %d not found, ignoring it", path[i]);
            }
        }

        return route;
    }
  
    /*------------------------------------------------------------------------------
    *  Name: run
    *      Just a test for buildRoute. Load waypoints from a given file and creates a route.
    *
    *---------------------------------------------------------------------------*/
    void run() {
        std::cout << "Available waypoints: " << std::endl;
        
        /* Print loaded waypoints */
        for(int i = 0; i < waypoints_.waypoints.size(); i++) {
            std::cout << "Waypoint ID: " << waypoints_.waypoints[i].id << std::endl;
        }

        int path[] = {8, 2, 4, 1, 9, 3, 7, 5, 0, 6}; /* Example of a path */
        int waypoint_type[] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0};
        int pathLength = 10;

        cirkit_waypoint_manager_msgs::WaypointArray route = buildRoute(path, waypoint_type, pathLength);

        std::cout << std::endl;
        std::cout << "Route: " << std::endl;

        /* Print route */
        for(int i = 0; i < route.waypoints.size(); i++) {
            std::cout << "Waypoint ID: " << route.waypoints[i].id << std::endl;
            std::cout << "Waypoint Type: " << route.waypoints[i].is_inspection_area << std::endl;
            std::cout << "    " << "x: " << route.waypoints[i].pose.position.x << std::endl;
            std::cout << "    " << "y: " << route.waypoints[i].pose.position.y << std::endl;
            std::cout << "    " << "z: " << route.waypoints[i].pose.position.z << std::endl;
            std::cout << "    " << "latitude: " << route.waypoints[i].gps_fix.latitude << std::endl;
            std::cout << "    " << "longitude: " << route.waypoints[i].gps_fix.longitude << std::endl;
            std::cout << "    " << "altitude: " << route.waypoints[i].gps_fix.altitude << std::endl;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Rate rate_;

    cirkit_waypoint_manager_msgs::WaypointArray waypoints_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "route");
    RouteGenerator route_generator;

    route_generator.run();

    return 0;
}
