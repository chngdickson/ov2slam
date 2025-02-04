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
#include <sensor_msgs/Imu.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "ov2slam.hpp"
#include "slam_params.hpp"


// 1. TF sync
// 2. Send 1 Camera to ov2slam for bundle adjustment of trajectories
// 3. Subscribe a service callback of trajectory generated from ov2slam
// 4. Get the sync TF from syncing the trajectories via the rosbag
// 5. 

// This needs
// 1. Subscriber and publisher of a single camera for framerate reduction
// 2. Service for ov2slam trajectory
// 3. Publisher to voxblox in terms of pointcloud
// 4. Subscribe to all the cameras and respective poses for pointcloud generation

void cam_left_cb(const sensor_msgs::Image::ConstPtr& pcleft_msg){
    ROS_INFO("hello x2")
};
void cam_cb(
  const sensor_msgs::Image::ConstPtr& pcleft_msg,
  const sensor_msgs::Image::ConstPtr& pcright_msg
){
    ROS_INFO("helo");
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filter_node");
    ros::NodeHandle nh;

    //Normal subscribers
    // ros::Subscriber cam_left_info_sub = nh->subscribe("/carla/ego_vehicle/depth_back/image", 1, cam_left_cb);
    // ros::Subscriber cam_right_info_sub = nh->subscribe("/carla/ego_vehicle/depth_back_left/image", 1, cam_right_cb);

    //Msg filter subscribers
    message_filters::Subscriber<sensor_msgs::Image> cam_left_sub(nh, "/carla/ego_vehicle/depth_back/image", 5);
    message_filters::Subscriber<sensor_msgs::Image> cam_right_sub(nh, "/carla/ego_vehicle/depth_back_left/image", 5);
    message_filters::TimeSynchronizer <sensor_msgs::Image, sensor_msgs::Image> ros_sync(
        cam_left_sub, cam_right_sub, 10
        );
    ros_sync.registerCallback(boost::bind(&cam_cb, _1, _2));

    // Spin and cleanup
    ros::spin();
    return 0;
}