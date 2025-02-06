#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <depth_image_proc/depth_traits.h>
#include "ov2slam.hpp"
#include "slam_params.hpp"
/*
1. TF sync
2. Send 1 Camera to ov2slam for bundle adjustment of trajectories
3. Subscribe a service callback of trajectory generated from ov2slam
4. Get the sync TF from syncing the trajectories via the rosbag
5. 

This needs
1. Subscriber and publisher of a single camera for framerate reduction
2. Service for ov2slam trajectory
3. Publisher to voxblox in terms of pointcloud
4. Subscribe to all the cameras and respective poses for pointcloud generation
*/



void cam_cb(
    const sensor_msgs::Image::ConstPtr& cam1,
    const sensor_msgs::Image::ConstPtr& cam2,
    const sensor_msgs::Image::ConstPtr& cam3,
    const sensor_msgs::Image::ConstPtr& cam4,
    const sensor_msgs::Image::ConstPtr& cam5,
    const sensor_msgs::Image::ConstPtr& cam6
){
    // auto cloud_msg =  std::make_unique<PointCloud2>();
    ROS_INFO("helo");
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh;

    //Normal subscribers
    // ros::Subscriber cam_left_info_sub = nh.subscribe("/carla/ego_vehicle/depth_back/image", 1, cam_left_cb);
    // ros::Subscriber cam_right_info_sub = nh->subscribe("/carla/ego_vehicle/depth_back_left/image", 1, cam_right_cb);

    //Msg filter subscribers
    // message_filters::Subscriber<sensor_msgs::Image> cam1(nh, "/carla/ego_vehicle/depth_back/image", 5);
    // message_filters::Subscriber<sensor_msgs::Image> cam2(nh, "/carla/ego_vehicle/depth_back_left/image", 5);
    // message_filters::Subscriber<sensor_msgs::Image> cam3(nh, "/carla/ego_vehicle/depth_back_right/image", 5);
    // message_filters::Subscriber<sensor_msgs::Image> cam4(nh, "/carla/ego_vehicle/depth_front/image", 5);
    // message_filters::Subscriber<sensor_msgs::Image> cam5(nh, "/carla/ego_vehicle/depth_front_left/image", 5);
    // message_filters::Subscriber<sensor_msgs::Image> cam6(nh, "/carla/ego_vehicle/depth_front_right/image", 5);
    // message_filters::TimeSynchronizer <sensor_msgs::Image, sensor_msgs::Image> ros_sync(
    //     cam1,cam2,cam3,cam4,cam5,cam6, 10
    //     );
    // ros_sync.registerCallback(boost::bind(&cam_cb, _1, _2));
    message_filters::Subscriber<sensor_msgs::Image> depth(nh, "/carla/ego_vehicle/depth_back/image",5);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info(nh, "/carla/ego_vehicle/depth_back/camera_info", 5);
    message_filters::TimeSynchronizer <sensor_msgs::Image, sensor_msgs::CameraInfo> ros_sync(
        depth, cam_info, 10
        );
    ros_sync.registerCallback(boost::bind(&cam_cb, _1, _2));

    // Spin and cleanup
    ros::spin();
    return 0;
}