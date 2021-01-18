#!/usr/bin/env python

import rospy
from tf_conversions import transformations
import requests

# ------------------------------------------------------------------------------
#   Name: SendWaypoints
#      Class which implements methods for loading waypoints from *.csv file and
#      sending them over a http request.
#
# ------------------------------------------------------------------------------
class SendWaypoints:
    # ------------------------------------------------------------------------------
    #   Name: __init__
    #      Class constructor. This function inits the node and loads the parameter
    #      which indicates the path to the waypoint file. 
    #
    # ------------------------------------------------------------------------------
    def __init__(self):
        self.waypoints_path = ''

        rospy.init_node('sendWaypoints', anonymous=False)

        if rospy.has_param("~waypoints_file"):
            self.waypoints_path = rospy.get_param("~waypoints_file")
            rospy.loginfo(self.waypoints_path)
        else:
            rospy.logerr("Parameter waypoints_file not informed")
            exit(-1)

    # ------------------------------------------------------------------------------
    #   Name: run
    #      This method loads waypoints from a file and send them to the server
    #
    # ------------------------------------------------------------------------------
    def run(self):
        waypoints_data = self.loadWaypoints(self.waypoints_path)
        if waypoints_data is not None:
            self.send(waypoints_data)

    # ------------------------------------------------------------------------------
    #   Name: loadWaypoints
    #      This function opens the csv file and load the waypoint's data inside it.
    #
    #       - waypoints_path: Path to the csv file
    #
    #   Return's description: Array where each row corresponds to a csv row.
    #
    # ------------------------------------------------------------------------------
    def loadWaypoints(self, waypoints_path):
        waypoints_data = []
        rospy.loginfo("Loading {0}".format(self.waypoints_path))
        with open(waypoints_path, 'r') as waypoints_file:
            for line in waypoints_file:
                line = line.replace("\n", "")
                row = line.split(",")
                if len(row) != 25:
                    rospy.logerr("Inconsistent row length")
                    return None
                else:
                    waypoints_data.append(row)

        return waypoints_data

    # ------------------------------------------------------------------------------
    #   Name: send
    #      This function gets loaded data, mounts a dict for each waypoint and
    #      sends a list of waypoints to the server.
    #
    #       - waypoints_data: Data loaded from a csv file
    #
    # ------------------------------------------------------------------------------
    def send(self, waypoints_data):
        waypoints_payload = []
        for row in waypoints_data:
            waypoint_dict = {}
            waypoint_dict["name"] = "Ponto " + str(row[0]) # "Ponto ID"
            waypoint_dict["lat"] = float(row[10])
            waypoint_dict["lng"] = float(row[11])
            waypoint_dict["map_id"] = 1
            waypoint_dict["x"] = float(row[1])
            waypoint_dict["y"] = float(row[2])
            waypoint_dict["teta"] = transformations.euler_from_quaternion([row[4], row[5], row[6], row[7]])[2]  # Converts quaternion to RPY (teta = yaw)

            waypoints_payload.append(waypoint_dict)

        requests.post("http://localhost:5002/waypoints", json=waypoints_payload) # TODO: Update with the real endpoit
        rospy.loginfo("Waypoints sent")


if __name__ == '__main__':
    sendWaypoints = SendWaypoints()
    sendWaypoints.run()
