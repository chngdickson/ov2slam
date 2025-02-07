#!/usr/bin/env python3
import rospy
import torch
import tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import message_filters


class CarlaSyncListener:
    def __init__(self):
        self.image_sub = message_filters.Subscriber('carla/ego_vehicle/rgb_back/image', Image)
        self.info_sub = message_filters.Subscriber('carla/ego_vehicle/rgb_back/camera_info', CameraInfo)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.info_sub], 1, 1) # Changed code
        self.ts.registerCallback(self.callback)

    def callback(self, image, camera_info):
        print("done")
    
if __name__ == '__main__':
    csl = CarlaSyncListener()
    rospy.init_node("sample_message_filters", anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("shutting_down")