#!/usr/bin/env python
import rospy
import torch
import numpy as np
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
        return (timestamp in self.timestampedInfo, self.timestampedInfo.get(timestamp))

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
        istrue, rgb_Rgbinfo_Depths = [],[]
        for csl in self.listenerDict.values():
            k, val = csl.timeStampExist(timestamp)
            istrue.append(k), rgb_Rgbinfo_Depths.append(val)
        if all(istrue):
            for rgb, info, depth in rgb_Rgbinfo_Depths:
                # 1. Test Depth to pcd
                # 2. 
                self.process_depthRgbc(None, None, depthImg=depth.data, conf=info)
            rospy.loginfo("message filter called, all infos exists")
    
    def process_depthRgbc(self, rgbImg, semImg, depthImg, conf:CameraInfo, camExt2WorldRH):
        pcd_np_3d = self.depthImg2Pcd(depthImg, w=conf.width, h=conf.height, K=conf.K)
        # Transform Lidar_np_3d from Camera To World Frame
        # pcd_np_3d = np.dot(camExt2WorldRH, np.vstack([pcd_np_3d, np.ones(pcd_np_3d.shape[1])]))[:3]
        # rgbSemCombi = np.dstack((rgbImg, semImg))
        # rgbSemCombi = np.reshape(rgbSemCombi, (rgbSemCombi.shape[0]*rgbSemCombi.shape[1], rgbSemCombi.shape[2]))
        # return np.hstack([pcd_np_3d.T, rgbSemCombi])
    
    def depthImg2Pcd(self, normalized_depth, w, h, K, max_depth=0.9):
        """
        Convert an image containing CARLA encoded depth-map to a 2D array containing
        the 3D position (relative to the camera) of each pixel and its corresponding
        RGB color of an array.
        "max_depth" is used to omit the points that are far enough.
        """
        far = 1000.0  # max depth in meters.
        w,h,K = int(w), int(h), np.array(K)
        pixel_length = w*h
        u_coord = np.matlib.repmat(np.r_[w-1:-1:-1],
                        h, 1).reshape(pixel_length)
        v_coord = np.matlib.repmat(np.c_[h-1:-1:-1],
                        1, w).reshape(pixel_length)
        normalized_depth = np.reshape(normalized_depth, pixel_length)
        # Search for pixels where the depth is greater than max_depth to
        # Make them = 0 to preserve the shape
        max_depth_indexes = np.where(normalized_depth > max_depth)
        normalized_depth[max_depth_indexes] = 0
        u_coord[max_depth_indexes] = 0
        v_coord[max_depth_indexes] = 0
        depth_np_1d = normalized_depth *far

        # p2d = [u,v,1]
        p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])

        # P = [X,Y,Z] # Pixel Space to Camera space
        p3d = np.dot(np.linalg.inv(K), p2d)
        p3d *= depth_np_1d

        lidar_np_3d = np.transpose(p3d) 
        py,pz,px = lidar_np_3d[:, 0], lidar_np_3d[:, 1], lidar_np_3d[:, 2]

        lidar_np_3d = np.vstack((px,py,pz))
        
        return lidar_np_3d
    
if __name__ == '__main__':
    # csl = CarlaSyncListener("back")
    msl = ManySyncListener()
    rospy.init_node("sample_message_filters", anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("shutting_down")