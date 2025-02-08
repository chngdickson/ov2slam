#!/usr/bin/env python

# Python 
import threading
from typing import Dict
from collections import OrderedDict

# Other libs
import torch, gc
import cv2
import numpy as np
import numpy.matlib as npm

# ROS
import rospy
import message_filters
from tf import TransformListener, transformations
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, PointField

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

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
        self.timer = rospy.Timer(rospy.Duration(0.01), self.wait_tf_cb)
        
        # Private vars
        self.extrinsic_to_origin = None
        
    def callback(self, rgb_img:Image, camera_info:CameraInfo, depth_img:Image):
        if not self.tf_received:
            self.tf_rel_frame = rgb_img.header.frame_id
            self.tf_rel_frame2 = depth_img.header.frame_id
            return
        self.timestampedInfo[rgb_img.header.stamp] = [rgb_img, camera_info, depth_img]
        if len(self.timestampedInfo) >= 5:
            self.timestampedInfo.popitem(False)
    
    def timeStampExist(self, timestamp):
        if timestamp in self.timestampedInfo:
            data = self.timestampedInfo.get(timestamp)
            self.timestampedInfo.pop(timestamp)
            return (True, data)
        else:
            return (False, None)

    def wait_tf_cb(self, event):
        if self.tf_rel_frame is None or self.tf_rel_frame2 is None:
            return
        else:
            self.check_tf_exists(self.tf_origin_frame, self.tf_rel_frame)

    def check_tf_exists(self, origin_frame, relative_frame):
        if self.tf_listener.frameExists(relative_frame):
            t = self.tf_listener.getLatestCommonTime(origin_frame, relative_frame)
            position, quaternion = self.tf_listener.lookupTransform(origin_frame, relative_frame, t)
            quat = transformations.quaternion_matrix(quaternion)
            quat[0:3,3] = position
            self.tf_received, self.extrinsic_to_origin = True, quat
            rospy.loginfo(f"{self.topic_pose}")
            self.timer.shutdown()
            
class ManySyncListener:
    def __init__(self):
        topics_list = ["front", "front_left", "front_right", "back", "back_left","back_right"]
        self.listenerDict:Dict[str,CarlaSyncListener] = {n:CarlaSyncListener(n) for n in topics_list}
        
        # message_filters Synchronizer
        self.list_filters = [message_filters.Subscriber(f"carla/ego_vehicle/rgbd_{n}/image", Image) for n in topics_list]
        self.ts = message_filters.TimeSynchronizer(self.list_filters, 10)
        self.ts.registerCallback(self.time_stamp_fuse_cb)

        # Publisher
        self.pc2_pub = rospy.Publisher("ego_vehicle_pcd",PointCloud2, queue_size=10)
        
    def time_stamp_fuse_cb(self, 
        front:Image, front_left:Image, front_right:Image, 
        back:Image, back_left:Image, back_right:Image
        ):
        rospy.loginfo("message filter called")
        timestamp = front.header.stamp
        istrues, rgb_Rgbinfo_Depths, ext_list = [],[],[]
        for csl in self.listenerDict.values():
            trueFalse, data = csl.timeStampExist(timestamp)
            istrues.append(trueFalse), rgb_Rgbinfo_Depths.append(data), ext_list.append(csl.extrinsic_to_origin) # type: ignore 
        if all(istrues):
            xyzrgb_list = []
            for (rgb, cam_info, depth),(ext2_Origin) in zip(rgb_Rgbinfo_Depths, ext_list):
                # 1. TODO: Test Depth to pcd
                # 2. TODO: Test pointcloud visualization 
                xyzrgb_list.append(self.process_depthRgbc(rgb, depth, cam_info, ext2_Origin))
            xyzrgb = np.dstack(xyzrgb_list)
            self.publish_pcd(xyzrgb, timestamp)
            rospy.loginfo("message filter called, all infos exists")
    
    
    def publish_pcd(self, arr, stamp):
        msg = PointCloud2()
        header = Header()
        header.frame_id = "ego_vehicle"
        header.stamp = stamp

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r',offset=12, datatype=PointField.UINT32, count=1),
            PointField(name='g',offset=16, datatype=PointField.UINT32, count=1),
            PointField(name='b',offset=20, datatype=PointField.UINT32, count=1),
        ]

        arr = arr.reshape(6,-1).T
        pc2 = point_cloud2.create_cloud(header, fields, arr)
        self.pc2_pub.publish(pc2)

    def process_depthRgbc(self, rgbImg, depthImg, conf:CameraInfo, camExt2WorldRH):
        pcd_np_3d = self.depthImg2Pcd(self.ros_depth_img2numpy(depthImg), w=conf.width, h=conf.height, K_ros=conf.K, ExtCam2Ego=camExt2WorldRH)
        pcd_np_3d = pcd_np_3d.detach().cpu().numpy()
        rgb = self.ros_rgb_img2numpy(rgbImg)
        a = np.vstack((pcd_np_3d, rgb)).reshape(6,-1)
        return a

    def ros_rgb_img2numpy(self, rgb_img: Image):
        im = np.frombuffer(rgb_img.data, dtype=np.uint8).reshape(-1, rgb_img.height, rgb_img.width)
        # BGRA to RGB
        im = im[:3,:,:]
        im = im[::-1,:,:]
        return im
    
    def ros_depth_img2numpy(self, ros_img: Image) -> np.ndarray:
        array = np.frombuffer(ros_img.data, dtype=np.float32)
        array = np.reshape(array, (ros_img.height, ros_img.width))
        array = cv2.normalize(array, None, 0, 1, cv2.NORM_MINMAX)
        return array

    def K3x3to4x4(self,K:torch.Tensor)->torch.Tensor:
        """Change the shape of K to 4x4 for easier matrix multiplication

        Args:
            K (torch.Tensor): _description_

        Returns:
            torch.Tensor: _description_
        """
        if len(K.shape) == 3:
            B = K.shape[0]
            K4x4 = torch.zeros(B, 4, 4, dtype=K.dtype, device=K.device)
            K4x4[:,:3,:3] = K
            K4x4[:,-1,-1] = 1.0
        elif len(K.shape) == 4:
            B, NCams, _, _ = K.shape
            K4x4 = torch.zeros(B, NCams, 4, 4, dtype=K.dtype, device=K.device)
            K4x4[:,:, :3,:3] = K
            K4x4[:,:,-1,-1] = 1.0
        elif len(K.shape) == 2:
            K4x4 = torch.zeros(4, 4, dtype=K.dtype, device=K.device)
            K4x4[:3,:3] = K
            K4x4[-1,-1] = 1.0
        else:
            raise ValueError(f"K3x3to4x4: Invalid shape of K: {K.shape}")
        return K4x4
    
    def depthImg2Pcd(self, normalized_depth, w, h, K_ros, max_depth=0.9, ExtCam2Ego=None):
        """
        Convert an image containing CARLA encoded depth-map to a 2D array containing
        the 3D position (relative to the camera) of each pixel and its corresponding
        RGB color of an array.
        "max_depth" is used to omit the points that are far enough.
        """
        far = 1000.0  # max depth in meters.
        w,h = int(w), int(h)
        pixel_length = w*h
        normalized_depth = torch.tensor(normalized_depth).to(device).reshape(pixel_length)
        dtype = normalized_depth.dtype
        K4x4 = self.K3x3to4x4(torch.tensor(K_ros).reshape((3,3))).to(device=device, dtype=dtype)
        
        M_Basis_Cam2W = torch.tensor([
                            [ 0, 1, 0, 0],
                            [ 0, 0, 1, 0],
                            [ 1, 0, 0, 0],
                            [ 0, 0, 0, 1]], dtype=dtype, device=device)
        
        u_coord = ((torch.arange(w-1, -1, -1).to(device).unsqueeze(0)).repeat(h,1)).reshape(pixel_length)
        v_coord = ((torch.arange(h-1, -1, -1).to(device).unsqueeze(1)).repeat(1,w)).reshape(pixel_length)
        
        # Search for pixels where the depth is greater than max_depth 
        # Make them = 0 to preserve the shape
        max_depth_indexes = torch.where(normalized_depth > max_depth)
        normalized_depth[max_depth_indexes], u_coord[max_depth_indexes], v_coord[max_depth_indexes] = 0,0,0
        normalized_depth = normalized_depth * far

        # p2d = [u,v,1]
        if ExtCam2Ego is not None:
            ExtCam2Ego = torch.tensor(ExtCam2Ego).to(device=device, dtype=dtype)
            pixel2WorldProjection = torch.pinverse(K4x4 @ M_Basis_Cam2W @ torch.pinverse(ExtCam2Ego))
        else:
            pixel2WorldProjection = torch.pinverse(K4x4 @ M_Basis_Cam2W)
            
        p3d = torch.vstack(
            [u_coord*normalized_depth,
             v_coord*normalized_depth,
             torch.ones_like(u_coord)*normalized_depth, 
             torch.ones_like(u_coord)]
            ).to(dtype)
        p3d = (- (pixel2WorldProjection @ p3d)[:3,:]).reshape(3, h, w)
        del v_coord, u_coord, normalized_depth, max_depth_indexes, ExtCam2Ego, K4x4
        torch.cuda.empty_cache()
        return p3d

if __name__ == '__main__':
    rospy.init_node("sample_message_filters", anonymous=True)
    
    msl = ManySyncListener()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        del msl
        torch.cuda.empty_cache()
        gc.collect()
        
        print("shutting_down")