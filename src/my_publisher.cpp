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

void cam_left_cb(const sensor_msgs::Image::ConstPtr& pcleft_msg){
    ROS_INFO("hello x2");
};
template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::ImagePtr& registered_msg,
                              const Eigen::Affine3d& depth_to_rgb)
{
  // Allocate memory for registered depth image
  registered_msg->step = registered_msg->width * sizeof(T);
  registered_msg->data.resize( registered_msg->height * registered_msg->step );
  // data is already zero-filled in the uint16 case, but for floats we want to initialize everything to NaN.
  DepthTraits<T>::initializeBuffer(registered_msg->data);

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_model_.fx();
  double inv_depth_fy = 1.0 / depth_model_.fy();
  double depth_cx = depth_model_.cx(), depth_cy = depth_model_.cy();
  double depth_Tx = depth_model_.Tx(), depth_Ty = depth_model_.Ty();
  double rgb_fx = rgb_model_.fx(), rgb_fy = rgb_model_.fy();
  double rgb_cx = rgb_model_.cx(), rgb_cy = rgb_model_.cy();
  double rgb_Tx = rgb_model_.Tx(), rgb_Ty = rgb_model_.Ty();
  
  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  T* registered_data = reinterpret_cast<T*>(&registered_msg->data[0]);
  int raw_index = 0;
  for (unsigned v = 0; v < depth_msg->height; ++v, depth_row += row_step)
  {
    for (unsigned u = 0; u < depth_msg->width; ++u, ++raw_index)
    {
      T raw_depth = depth_row[u];
      if (!DepthTraits<T>::valid(raw_depth))
        continue;
      
      double depth = DepthTraits<T>::toMeters(raw_depth);

      if (fill_upsampling_holes_ == false)
      {
        /// @todo Combine all operations into one matrix multiply on (u,v,d)
        // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
        Eigen::Vector4d xyz_depth;
        xyz_depth << ((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                     ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                     depth,
                     1;

        // Transform to RGB camera frame
        Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

        // Project to (u,v) in RGB image
        double inv_Z = 1.0 / xyz_rgb.z();
        int u_rgb = (rgb_fx*xyz_rgb.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
        int v_rgb = (rgb_fy*xyz_rgb.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
      
        if (u_rgb < 0 || u_rgb >= (int)registered_msg->width ||
            v_rgb < 0 || v_rgb >= (int)registered_msg->height)
          continue;
      
        T& reg_depth = registered_data[v_rgb*registered_msg->width + u_rgb];
        T  new_depth = DepthTraits<T>::fromMeters(xyz_rgb.z());
        // Validity and Z-buffer checks
        if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth)
          reg_depth = new_depth;
      }
      else
      {
        // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
        Eigen::Vector4d xyz_depth_1, xyz_depth_2;
        xyz_depth_1 << ((u-0.5f - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                       ((v-0.5f - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                       depth,
                       1;
        xyz_depth_2 << ((u+0.5f - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                       ((v+0.5f - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                       depth,
                       1;

        // Transform to RGB camera frame
        Eigen::Vector4d xyz_rgb_1 = depth_to_rgb * xyz_depth_1;
        Eigen::Vector4d xyz_rgb_2 = depth_to_rgb * xyz_depth_2;

        // Project to (u,v) in RGB image
        double inv_Z = 1.0 / xyz_rgb_1.z();
        int u_rgb_1 = (rgb_fx*xyz_rgb_1.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
        int v_rgb_1 = (rgb_fy*xyz_rgb_1.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;
        inv_Z = 1.0 / xyz_rgb_2.z();
        int u_rgb_2 = (rgb_fx*xyz_rgb_2.x() + rgb_Tx)*inv_Z + rgb_cx + 0.5;
        int v_rgb_2 = (rgb_fy*xyz_rgb_2.y() + rgb_Ty)*inv_Z + rgb_cy + 0.5;

        if (u_rgb_1 < 0 || u_rgb_2 >= (int)registered_msg->width ||
            v_rgb_1 < 0 || v_rgb_2 >= (int)registered_msg->height)
          continue;

        for (int nv=v_rgb_1; nv<=v_rgb_2; ++nv)
        {
          for (int nu=u_rgb_1; nu<=u_rgb_2; ++nu)
          {
            T& reg_depth = registered_data[nv*registered_msg->width + nu];
            T  new_depth = DepthTraits<T>::fromMeters(0.5*(xyz_rgb_1.z()+xyz_rgb_2.z()));
            // Validity and Z-buffer checks
            if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth)
              reg_depth = new_depth;
          }
        }
      }
    }
  }
}

void convertdepth_cb(
  const sensor_msgs::Image::ConstPtr& depth_msg,
  const sensor_msgs::CameraInfo::ConstPtr& info_msg
){
  auto cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  image_geometry::PinholeCameraModel model_;
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1 || depth_msg->encoding == enc::MONO16)
  {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_msg, cloud_msg, model_);
  }
  else
  {
    ROS_ERROR("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }
  // Convert Depth Image to Pointcloud
//   if (depth_msg->encoding == enc::TYPE_16UC1 || depth_msg->encoding == enc::MONO16) {
//     convertDepth<uint16_t>(depth_msg, *cloud_msg, model_, invalid_depth_);
//   } else if (depth_msg->encoding == enc::TYPE_32FC1) {
//     convertDepth<float>(depth_msg, *cloud_msg, model_, invalid_depth_);
//   } else {
//     ROS_ERROR(
//       get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
//     return;
//   }

//   pub_point_cloud_->publish(std::move(cloud_msg));
};

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
    ros_sync.registerCallback(boost::bind(&convertdepth_cb, _1, _2));

    // Spin and cleanup
    ros::spin();
    return 0;
}