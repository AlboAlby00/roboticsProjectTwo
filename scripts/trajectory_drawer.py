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
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import os
from robotics_project_two.srv import CreateMapAndTrajectoryImage, ResetTrajectoryToDraw
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import os

cwd = os.getcwd()

class BaseMap:
    def __init__(self,map,width,height,resolution,origin):
        rospy.INFO("creating base map")
        self.map = map
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.trajectory = []
        self.poseSubscriber = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.trajectory_callback)
        self.drawService = rospy.Service('create_map_and_trajectory_image', CreateMapAndTrajectoryImage, self.service_callback)
        self.resetService = rospy.Service('reset_trajectory_to_draw',ResetTrajectoryToDraw,self.reset_trajectory_callback)

    def trajectory_callback(self,data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        cellX = (x-self.origin.position.x)/self.resolution
        cellY = (y-self.origin.position.x)/self.resolution
        self.trajectory.append((cellX,cellY))

    def service_callback(self,data):
        size = (self.width,self.height)
        image = np.zeros(size)

        counter = 0
        for p in self.map:
            image[counter%self.width][int(counter/self.width)] = p
            counter+=1
        image[image == -1] = 150
        image[image == 0] = 255
        image[image == 100] = 0

        counter = 0
        color = (0, 255, 0)
        for (x,y) in self.trajectory:
            if(counter != 0):
                cv2.line(image,previous,(x,y),color,5)
            previous = (x,y)
            counter+=1

        image = cv2.rotate(image, cv2.ROTATE_180)
        
        filename = cwd+'/map_with_trajectory.png'
        print ("saving image")
        cv2.imwrite (filename,image)

    def reset_trajectory_callback(self,data):
        self.trajectory = []


def save_map_callback(data):
    rospy.INFO("salvo la mappa")
    global baseMap
    baseMap = BaseMap(data.data,data.info.width,data.info.height,data.info.resolution,data.info.origin)

def map_listener():
    rospy.init_node('trajectory_drawer',anonymous=False)
    rospy.wait_for_service('static_map')
    try:
        get_map = rospy.ServiceProxy('static_map',GetMap)
        map = get_map()
        save_map_callback(map.map)
        rospy.spin()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    

if __name__ == '__main__':
    map_listener()
