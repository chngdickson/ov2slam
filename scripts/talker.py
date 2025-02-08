#!/usr/bin/env python
import rospy
import torch
import numpy as np
import numpy.matlib as npm
import cv2
from tf import TransformListener
from typing import Dict
from collections import OrderedDict
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import message_filters
import threading

class CarlaSyncListener:
    def __init__(self, topic_pose, tf_origin_frame="ego_vehicle"):
        self.topic_pose = topic_pose
        self.image_sub = message_filters.Subscriber(f"carla/ego_vehicle/rgb_{topic_pose}/image", Image)
        self.info_sub = message_filters.Subscriber(f"carla/ego_vehicle/rgbd_{topic_pose}/camera_info", CameraInfo)
        self.depth_sub = message_filters.Subscriber(f"carla/ego_vehicle/rgbd_{topic_pose}/image", Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.callback)
        self.timestampedInfo = OrderedDict() #Timestamp:combined pointcloud
        
        # TF's
        self.tf_received = False
        self.tf_listener = TransformListener()
        self.tf_origin_frame, self.tf_rel_frame, self.tf_rel_frame2 = tf_origin_frame, None, None
        self.pose, self.quat = None, None
        self.timer = rospy.Timer(rospy.Duration(0.01), self.wait_tf_cb)
        
    def callback(self, rgb_img:Image, camera_info:CameraInfo, depth_img:Image):
        if not self.tf_received:
            self.tf_rel_frame = rgb_img.header.frame_id
            self.tf_rel_frame2 = depth_img.header.frame_id
            return
        self.timestampedInfo[rgb_img.header.stamp] = [rgb_img, camera_info, depth_img]
        if len(self.timestampedInfo) >= 5:
            self.timestampedInfo.popitem(False)
    
    def timeStampExist(self, timestamp):
        return (timestamp in self.timestampedInfo, self.timestampedInfo.get(timestamp))

    def wait_tf_cb(self, event):
        if self.tf_rel_frame is None:
            return
        print(self.tf_rel_frame)
        if self.tf_listener.frameExists(self.tf_rel_frame):
            t = self.tf_listener.getLatestCommonTime(self.tf_origin_frame, self.tf_rel_frame)
            position, quaternion = self.tf_listener.lookupTransform(self.tf_origin_frame, self.tf_rel_frame, t)
            self.tf_received, self.pose, self.quat = True, position, quaternion
            rospy.loginfo(f"{self.topic_pose}")
            self.timer.shutdown()
        if self.tf_rel_frame2 is None:
            return
        if self.tf_listener.frameExists(self.tf_rel_frame2):
            t = self.tf_listener.getLatestCommonTime(self.tf_origin_frame, self.tf_rel_frame2)
            position, quaternion = self.tf_listener.lookupTransform(self.tf_origin_frame, self.tf_rel_frame2, t)
            self.tf_received, self.pose, self.quat = True, position, quaternion
            rospy.loginfo(f"{self.topic_pose}")
            self.timer.shutdown()
class ManySyncListener:
    def __init__(self):
        topics_list = ["front", "front_left", "front_right", "back", "back_left","back_right"]
        self.listenerDict:Dict[str,CarlaSyncListener] = {n:CarlaSyncListener(n) for n in topics_list}
        self.list_filters = [message_filters.Subscriber(f"carla/ego_vehicle/rgbd_{n}/image", Image) for n in topics_list]
        self.ts = message_filters.TimeSynchronizer(self.list_filters, 10)
        self.ts.registerCallback(self.time_stamp_fuse_cb)
        self.tf = TransformListener()

    def time_stamp_fuse_cb(self, 
        front:Image, front_left:Image, front_right:Image, 
        back:Image, back_left:Image, back_right:Image
        ):
        rospy.loginfo("message filter called")
        timestamp = front.header.stamp
        istrues, rgb_Rgbinfo_Depths, pose_quat = [],[],[]
        for csl in self.listenerDict.values():
            trueFalse, data = csl.timeStampExist(timestamp)
            istrues.append(trueFalse), rgb_Rgbinfo_Depths.append(data), pose_quat.append([csl.pose, csl.quat])
        if all(istrues):
            for (rgb, info, depth),(pose, quat) in zip(rgb_Rgbinfo_Depths, pose_quat):
                # 1. Test Depth to pcd
                # 2. Test 
                self.process_depthRgbc(None, None, depthImg=depth, conf=info, camExt2WorldRH=None)
            rospy.loginfo("message filter called, all infos exists")
    
        
    def process_depthRgbc(self, rgbImg, semImg, depthImg, conf:CameraInfo, camExt2WorldRH):
        # print(depthImg.data.shape)
        pcd_np_3d = self.depthImg2Pcd(self.ros_depth_img2numpy(depthImg), w=conf.width, h=conf.height, K=conf.K)
        # Transform Lidar_np_3d from Camera To World Frame
        # pcd_np_3d = np.dot(camExt2WorldRH, np.vstack([pcd_np_3d, np.ones(pcd_np_3d.shape[1])]))[:3]
        # rgbSemCombi = np.dstack((rgbImg, semImg))
        # rgbSemCombi = np.reshape(rgbSemCombi, (rgbSemCombi.shape[0]*rgbSemCombi.shape[1], rgbSemCombi.shape[2]))
        # return np.hstack([pcd_np_3d.T, rgbSemCombi])

    def ros_depth_img2numpy(self, ros_img: Image) -> np.ndarray:
        array = np.frombuffer(ros_img.data, dtype=np.float32)
        array = np.reshape(array, (ros_img.height, ros_img.width))
        array = cv2.normalize(array, None, 0, 1, cv2.NORM_MINMAX)
        return array
    
    def depthImg2Pcd(self, normalized_depth, w, h, K, max_depth=0.9):
        """
        Convert an image containing CARLA encoded depth-map to a 2D array containing
        the 3D position (relative to the camera) of each pixel and its corresponding
        RGB color of an array.
        "max_depth" is used to omit the points that are far enough.
        """

        far = 1000.0  # max depth in meters.
        w,h,K = int(w), int(h), np.array(K).reshape((3,3))
        pixel_length = w*h
        u_coord = np.matlib.repmat(np.r_[w-1:-1:-1],
                        h, 1).reshape(pixel_length)
        u_coord_torch = ((torch.arange(w-1, -1, -1).unsqueeze(0)).repeat(h,1)).reshape(pixel_length)
        assert np.all(u_coord, u_coord_torch.numpy()), "errror"
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
    rospy.init_node("sample_message_filters", anonymous=True)
    
    msl = ManySyncListener()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("shutting_down")