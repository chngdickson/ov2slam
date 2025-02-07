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
        ros::NodeHandle nh_;
        message_filters::Subscriber<sensor_msgs::Image> depth_sub; 
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
        ros::Publisher depth_new_pub;
    public:
        ExampleRosClass(ros::NodeHandle* nodehandle, std::string depth_topic, std::string rgb_topic, std::string depth_topicnew):nh_(*nodehandle)
        { // constructor
            initializeSubscribers(depth_topic, rgb_topic); 
            initializePublishers(depth_topicnew);
        }

        void initializeSubscribers(std::string depth_topic, std::string rgb_topic)
        {
            depth_sub.subscribe(nh_, depth_topic, 10);
            rgb_sub.subscribe(nh_, rgb_topic, 10);
            message_filters::TimeSynchronizer <sensor_msgs::Image, sensor_msgs::Image> ros_sync(
                depth_sub, rgb_sub, 10
                ); 
            ros_sync.registerCallback(boost::bind(&ExampleRosClass::subscriberCallback, this));
        } 
        void initializePublishers(std::string depth_topic_new)
        {
            ROS_INFO("Initializing Publishers");
            depth_new_pub = nh_.advertise<sensor_msgs::Image>(depth_topic_new, 1, true); 
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