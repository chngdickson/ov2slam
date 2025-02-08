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
        self.topic_pose = topic_pose
        self.image_sub = message_filters.Subscriber(f"carla/ego_vehicle/rgb_{topic_pose}/image", Image)
        self.info_sub = message_filters.Subscriber(f"carla/ego_vehicle/rgb_{topic_pose}/camera_info", CameraInfo)
        self.depth_sub = message_filters.Subscriber(f"carla/ego_vehicle/depth_{topic_pose}/image", Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.callback)
        self.timestampedInfo = OrderedDict() #Timestamp:combined pointcloud
        # we need to know the quickest method for depth conversion
    def callback(self, image:Image, camera_info:CameraInfo, depth_img:Image):
        timestamp = image.header.stamp
        rospy.loginfo(f"{self.topic_pose}")
        self.timestampedInfo[timestamp] = [image, camera_info, depth_img]
        if len(self.timestampedInfo) >= 5:
            self.timestampedInfo.popitem(False)

class ManySyncListener:
    def __init__(self):
        topics_list = ["front", "front_left", "front_right", "back", "back_left","back_right"]
        img_depth_lists:List[CarlaSyncListener] = [CarlaSyncListener(n) for n in topics_list]
        self.list_filters = [message_filters.Subscriber(f"carla/ego_vehicle/depth_{n}/image", Image) for n in topics_list]
        self.ts = message_filters.TimeSynchronizer([self.list_filters], 10)
        self.ts.registerCallback(self.time_stamp_fuse_cb)
    
    def time_stamp_fuse_cb(self, front, front_left, front_right, back, back_left, back_right):
        rospy.loginfo("message filter called")
        pass
    
if __name__ == '__main__':
    csl = CarlaSyncListener("back")
    rospy.init_node("sample_message_filters", anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("shutting_down")