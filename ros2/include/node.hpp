#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include "orb_slam3_ros/SaveMap.hpp"


#include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.h>

#include "System.h"


namespace orb_slam3_ros{
class Node : public rclcpp::Node
{
  public:
    Node (ORB_SLAM3::System::eSensor sensor);
    Node (ORB_SLAM3::System::eSensor sensor, std::string name);
    ~Node ();
    void init ();

  protected:
    void update (Sophus::SE3f position);
    ORB_SLAM3::System* orb_slam_;
    rclcpp::Time current_frame_time_;
    std::string camera_info_topic_;

  private:
    void publish_map_points (std::vector<ORB_SLAM3::MapPoint*> map_points);
    void publish_position_as_transform (cv::Mat position);
    void publish_position_as_pose_stamped(cv::Mat position);
    void publish_gba_status (bool gba_status);
    void publish_rendered_image (cv::Mat image);


    bool save_map_srv (orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res);
    void loadOrbParameters (ORB_SLAM3::ORBParameters& parameters);

    // initialization Transform listener
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    //need to change to sophus rotation mat from cv
    tf2::Transform transform_from_mat (Sophus::SE3f position_mat);
    tf2::Transform transform_to_target (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    sensor_msgs::PointCloud2 map_points_to_point_cloud (std::vector<ORB_SLAM3::MapPoint*> map_points);

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_points_publisher_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_publisher_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::Bool>> status_gba_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> rendered_image_publisher_;

    std::shared_ptr<rclcpp::Service<orb_slam3_ros2::srv::SaveMap>> service_server_;

    std::string name_of_node_;


    ORB_SLAM3::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};
}

