#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
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

class ExampleRosClass{
    private:
        ros::NodeHandle _nh;
        message_filters::Subscriber<sensor_msgs::Image> _depth_sub; 
        message_filters::Subscriber<sensor_msgs::Image> _rgb_sub;
        using ExactSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
        std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> _sync;
        ros::Publisher depth_new_pub;
    
    public:
        ExampleRosClass(ros::NodeHandle* nodehandle, std::string depth_topic, std::string rgb_topic, std::string depth_topicnew):_nh(*nodehandle)
        { // constructor
            initializeSubscribers(depth_topic, rgb_topic); 
            initializePublishers(depth_topicnew);
        }

        void initializeSubscribers(std::string depth_topic, std::string rgb_topic)
        {
            _depth_sub.subscribe(_nh, depth_topic, 1);
            _rgb_sub.subscribe(_nh, rgb_topic, 1);
            _sync = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(10);
            _sync.connectInput(_depth_sub, _rgb_sub);
            _sync.registerCallback(boost::bind(&ExampleRosClass::subscriberCallback, this, _1, _2));
        } 
        void initializePublishers(std::string depth_topic_new)
        {
            ROS_INFO("Initializing Publishers");
            depth_new_pub = _nh.advertise<sensor_msgs::Image>(depth_topic_new, 1, true); 
        }
        void subscriberCallback(
            const sensor_msgs::Image::ConstPtr& depth_cam,
            const sensor_msgs::Image::ConstPtr& rgb_cam)
        {
            sensor_msgs::Image new_depth_msg = *depth_cam;
            new_depth_msg.header = rgb_cam->header;

            depth_new_pub.publish(new_depth_msg);
            ROS_INFO("are u publishing");
        }
};




int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "exampleRosClass"); //node name

    ros::NodeHandle nh; 

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    ExampleRosClass exampleRosClass(&nh, "/carla/ego_vehicle/depth_back/image", "/carla/ego_vehicle/rgb_back/image", "/carla/ego_vehicle/depth_back2/image");  

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 