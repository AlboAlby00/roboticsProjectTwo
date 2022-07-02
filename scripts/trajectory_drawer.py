#get map from mapserver topics /map at the beginning
#map is an nav_msgs/OccupancyGrid object and contains metadata "info"
#we get pose from topic amcl_pose
#compute the corresponding cell
#set occupancy to 100

#provide a service that takes the current occupancyGrid and saves it into a png file


#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
import os