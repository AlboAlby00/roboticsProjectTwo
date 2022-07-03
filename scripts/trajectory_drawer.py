#get map from mapserver topics /map at the beginning
#map is an nav_msgs/OccupancyGrid object and contains metadata "info"
#we get pose from topic amcl_pose
#compute the corresponding cell
#set occupancy to 100

#provide a service that takes the current occupancyGrid and saves it into a png file


#!/usr/bin/env python
from http import server
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
import os
from robotics_project_two.srv import CreateMapAndTrajectoryImage

def create_map_and_trajectory_image_function(req):
    print("service called")

def main():
    rospy.init_node('trajectory_drawer')
    s = rospy.Service('create_map_and_trajectory_image', CreateMapAndTrajectoryImage, create_map_and_trajectory_image_function)
    print("service is ready")
    rospy.spin()

if __name__ == "__main__":
    main()