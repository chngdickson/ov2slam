#!/usr/bin/env python
import rospy
import torch
import tf
from typing import List
from collections import OrderedDict
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import message_filters


class CarlaSyncListener:
    def __init__(self, topic_pose):
        self.image_sub = message_filters.Subscriber("carla/ego_vehicle/rgb_{topic_pose}/image", Image)
        self.info_sub = message_filters.Subscriber("carla/ego_vehicle/rgb_{topic_pose}/camera_info", CameraInfo)
        self.depth_sub = message_filters.Subscriber("carla/ego_vehicle/depth_{topic_pose}/image", Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.callback)
        self.timestampedInfo = OrderedDict() #Timestamp:combined pointcloud
        # we need to know the quickest method for depth conversion
    def callback(self, image:Image, camera_info:CameraInfo, depth_img:Image):
        timestamp = image.header.stamp
        self.timestampedInfo[timestamp] = [image, camera_info.k, depth_img]
        
        if len(self.timestampedInfo) >= 5:
            self.timestampedInfo.popitem(False)

class ManySyncListener:
    def __init__(self):
        img_depth_lists:List[CarlaSyncListener] = [
            CarlaSyncListener("back"),
            CarlaSyncListener("front")
        ]
        str_list = []
    def time_stamp_fuse(self):
        pass
    
if __name__ == '__main__':
    csl = CarlaSyncListener()
    rospy.init_node("sample_message_filters", anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("shutting_down")