#!/usr/bin/env python
import rospy
import torch
import tf
from typing import Dict
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
    
    def timeStampExist(self, timestamp):
        return [timestamp in self.timestampedInfo, self.timestampedInfo.get(timestamp)]

class ManySyncListener:
    def __init__(self):
        topics_list = ["front", "front_left", "front_right", "back", "back_left","back_right"]
        self.listenerDict:Dict[str,CarlaSyncListener] = {n:CarlaSyncListener(n) for n in topics_list}
        self.list_filters = [message_filters.Subscriber(f"carla/ego_vehicle/depth_{n}/image", Image) for n in topics_list]
        self.ts = message_filters.TimeSynchronizer(self.list_filters, 10)
        self.ts.registerCallback(self.time_stamp_fuse_cb)
    
    def time_stamp_fuse_cb(self, 
        front:Image, front_left:Image, front_right:Image, 
        back:Image, back_left:Image, back_right:Image
        ):
        rospy.loginfo("message filter called")
        timestamp = front.header.stamp
        a = map(list, zip([v.timeStampExist(timestamp)] for v in self.listenerDict.values()))
        print(len(a))
        # if all(istrue):
        #     rospy.loginfo("message filter called, all infos exists")
            
        
    
if __name__ == '__main__':
    # csl = CarlaSyncListener("back")
    msl = ManySyncListener()
    rospy.init_node("sample_message_filters", anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("shutting_down")