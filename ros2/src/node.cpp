#include "node.hpp"
#include "utils.hpp"
#include <iostream>
namespace orb_slam3_ros{
Node::Node (ORB_SLAM3::System::eSensor sensor) :  rclcpp::Node("orbslam") {
  min_observations_per_point_ = 2;
  sensor_ = sensor;
  name_of_node_ = "orbslam";
}

Node::Node (ORB_SLAM3::System::eSensor sensor, std::string name) :  rclcpp::Node(name) {
  min_observations_per_point_ = 2;
  sensor_ = sensor;
  name_of_node_ = name;
}

Node::~Node(){
    RCLCPP_INFO(this->get_logger(),"Closing orbslam node");
}

void Node::init (){
  //static parameters
  declare_parameter_if_not_declared(this,name_of_node_ + "/publish_pointcloud", rclcpp::ParameterValue(true));
  this->get_parameter(name_of_node_ + "/publish_pointcloud", publish_pointcloud_param_);
  
  
  declare_parameter_if_not_declared(this,name_of_node_ + "/publish_pose", rclcpp::ParameterValue(true));
  this->get_parameter(name_of_node_ +  "/publish_pose", publish_pose_param_);

  declare_parameter_if_not_declared(this,name_of_node_ + "/publish_tf", rclcpp::ParameterValue(true));
  this->get_parameter(name_of_node_ + "/publish_tf", publish_tf_param_);

  declare_parameter_if_not_declared(this,name_of_node_ + "/pointcloud_frame_id", rclcpp::ParameterValue("map"));
  this->get_parameter(name_of_node_ + "/pointcloud_frame_id", map_frame_id_param_);

  declare_parameter_if_not_declared(this,name_of_node_ + "/camera_frame_id", rclcpp::ParameterValue("camera_link"));
  this->get_parameter(name_of_node_ + "/camera_frame_id", camera_frame_id_param_);

  declare_parameter_if_not_declared(this,name_of_node_ + "/target_frame_id", rclcpp::ParameterValue("base_link"));
  this->get_parameter(name_of_node_ + "/target_frame_id", target_frame_id_param_);

  
  declare_parameter_if_not_declared(this,name_of_node_ + "/map_file", rclcpp::ParameterValue("map.bin"));
  this->get_parameter(name_of_node_ + "/map_file", map_file_name_param_);

  declare_parameter_if_not_declared(this,name_of_node_ + "/voc_file", rclcpp::ParameterValue("file_not_set"));
  this->get_parameter(name_of_node_ + "/voc_file", voc_file_name_param_);

  declare_parameter_if_not_declared(this,name_of_node_ + "/load_map", rclcpp::ParameterValue(false));
  this->get_parameter(name_of_node_ + "/load_map", load_map_param_);
  

   // Create a parameters object to pass to the Tracking system
   ORB_SLAM3::ORBParameters parameters;
   LoadOrbParameters (parameters);

  orb_slam_ = new ORB_SLAM3::System (voc_file_name_param_, sensor_, parameters, map_file_name_param_, load_map_param_);

  service_server_ = node->create_service<orb_slam3_ros2::srv::SaveMap>("/save_map", &Node::SaveMapSrv);

  //Setup dynamic reconfigure
  //   dynamic_reconfigure::Server<ORB_SLAM3_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  //   dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  //   dynamic_param_server_.setCallback(dynamic_param_callback);

  // Initialization transformation listener
  tfBuffer.reset(new tf2_ros::Buffer);
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  rendered_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(name_of_node_+"/debug_image", 1);

  if (publish_pointcloud_param_) {
    map_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (name_of_node_+"/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> (name_of_node_+"/pose", 1);
  }

  status_gba_publisher_ = this->create_publisher<std_msgs::msg::Bool> (name_of_node_+"/gba_running", 1);

}

void Node::Update(Sophus::SE3f position) {

  if (!position.empty()) {
    if (publish_tf_param_){
      PublishPositionAsTransform(position);
    }

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped(position);
    }
  }

  PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    publish_map_points (orb_slam_->GetAllMapPoints());
  }

  PublishGBAStatus (orb_slam_->isRunningGBA());

}

void Node::publish_map_points (std::vector<ORB_SLAM3::MapPoint*> map_points){
  sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}


void Node::publish_position_as_transform (Sophus::SE3f position){
  // Get transform from map to camera frame
  tf2::Transform tf_transform = transform_from_mat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_map2target = transform_to_target(tf_transform, camera_frame_id_param_, target_frame_id_param_);

  // Make message
  tf2::Stamped<tf2::Transform> tf_map2target_stamped;
  tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::msg::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
  msg.child_frame_id = target_frame_id_param_;
  // Broadcast tf
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(msg);    
}


// ------------------------------------------------------------------------
// geometry_msgs::Transform sophusToTransformMsg(const Sophus::SE3f& se3) {
//   geometry_msgs::Transform msg;
//   msg.translation.x = se3.translation().x();
//   msg.translation.y = se3.translation().y();
//   msg.translation.z = se3.translation().z();
//   msg.rotation.x = se3.unit_quaternion().x();
//   msg.rotation.y = se3.unit_quaternion().y();
//   msg.rotation.z = se3.unit_quaternion().z();
//   msg.rotation.w = se3.unit_quaternion().w();
//   return msg;
// }
// geometry_msgs::Pose sophusToPoseMsg(const Sophus::SE3f& s) {
//   geometry_msgs::Pose pose;
//   Eigen::Vector3f translation = s.translation();
//   pose.position = eigenToPointMsg(translation);
//   Eigen::Quaternionf quaternion = s.unit_quaternion();
//   pose.orientation = eigenToQuaternionMsg(quaternion);
//   return pose;
// }




tf2::Transform Node::transform_from_mat (Sophus::SE3f se3) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  transformation_matrix = se3.matrix()
  tf2::Matrix3x3 tf_camera_rotation (transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                    transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                    transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2)
                                   );

  tf2::Vector3 tf_camera_translation (se3.translation().x(), se3.translation().y(), se3.translation().z());

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

tf2::Transform Node::transform_to_target (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
  // Transform tf_in from frame_in to frame_target
  tf2::Transform tf_map2orig = tf_in;
  tf2::Transform tf_orig2target;
  tf2::Transform tf_map2target;

  tf2::Stamped<tf2::Transform> transformStamped_temp;
  try {
    // Get the transform from camera to target
    geometry_msgs::msg::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, ros::Time(0));
    // Convert to tf2
    tf2::fromMsg(tf_msg, transformStamped_temp);
    tf_orig2target.setBasis(transformStamped_temp.getBasis());
    tf_orig2target.setOrigin(transformStamped_temp.getOrigin());

  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    //ros::Duration(1.0).sleep();
    tf_orig2target.setIdentity();
  }

  // Transform from map to target
  tf_map2target = tf_map2orig * tf_orig2target;
  return tf_map2target;
}



bool Node::SaveMapSrv (orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res) {
  res.success = orb_slam_->SaveMap(req.name);

  if (res.success) {
    RCLCPP_INFO (this->get_logger(), "Map was saved as " << req.name);
  } else {
    RCLCPP_ERROR (this->get_logger(),"Map could not be saved.");
  }

  return res.success;
}


void Node::PublishPositionAsTransform (Sophus::SE3f position) {
  // Get transform from map to camera frame
  tf2::Transform tf_transform = transform_from_mat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_map2target = transform_to_target(tf_transform, camera_frame_id_param_, target_frame_id_param_);

  // Make message
  tf2::Stamped<tf2::Transform> tf_map2target_stamped;
  tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
  msg.child_frame_id = target_frame_id_param_;
  // Broadcast tf
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(msg);
}

void Node::PublishPositionAsPoseStamped (Sophus::SE3f position) {
  tf2::Transform tf_position = transform_from_mat(position);

  // Make transform from camera frame to target frame
  tf2::Transform tf_position_target = transform_to_target(tf_position, camera_frame_id_param_, target_frame_id_param_);
  
  // Make message
  tf2::Stamped<tf2::Transform> tf_position_target_stamped;
  tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, current_frame_time_, map_frame_id_param_);
  geometry_msgs::msg::PoseStamped pose_msg;
  tf2::toMsg(tf_position_target_stamped, pose_msg);
  pose_publisher_.publish(pose_msg);
}

void Node::PublishGBAStatus (bool gba_status) {
  std_msgs::msg::Bool gba_status_msg;
  gba_status_msg.data = gba_status;
  status_gba_publisher_.publish(gba_status_msg);
}

void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::msg::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


sensor_msgs::msg::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}


// loading rest of params via Tracking.cc for now

// void Node::LoadOrbParameters (ORB_SLAM3::ORBParameters& parameters) {
//   //ORB SLAM configuration parameters
//   node_handle_.param(name_of_node_ + "/camera_fps", parameters.maxFrames, 30);
//   node_handle_.param(name_of_node_ + "/camera_rgb_encoding", parameters.RGB, true);
//   node_handle_.param(name_of_node_ + "/ORBextractor/nFeatures", parameters.nFeatures, 1200);
//   node_handle_.param(name_of_node_ + "/ORBextractor/scaleFactor", parameters.scaleFactor, static_cast<float>(1.2));
//   node_handle_.param(name_of_node_ + "/ORBextractor/nLevels", parameters.nLevels, 8);
//   node_handle_.param(name_of_node_ + "/ORBextractor/iniThFAST", parameters.iniThFAST, 20);
//   node_handle_.param(name_of_node_ + "/ORBextractor/minThFAST", parameters.minThFAST, 7);

//   bool load_calibration_from_cam = false;
//   node_handle_.param(name_of_node_ + "/load_calibration_from_cam", load_calibration_from_cam, false);

//   if (sensor_== ORB_SLAM3::System::STEREO || sensor_==ORB_SLAM3::System::RGBD) {
//     node_handle_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
//     node_handle_.param(name_of_node_ + "/depth_map_factor", parameters.depthMapFactor, static_cast<float>(1.0));
//   }

//   if (load_calibration_from_cam) {
//     RCLCPP_INFO (this->get_logger(),"Listening for camera info on topic " << node_handle_.resolveName(camera_info_topic_));
//     sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, ros::Duration(1000.0));
//     if(camera_info == nullptr){
//         RCLCPP_WARN("Did not receive camera info before timeout, defaulting to launch file params.");
//     } else {
//       parameters.fx = camera_info->K[0];
//       parameters.fy = camera_info->K[4];
//       parameters.cx = camera_info->K[2];
//       parameters.cy = camera_info->K[5];

//       parameters.baseline = camera_info->P[3];

//       parameters.k1 = camera_info->D[0];
//       parameters.k2 = camera_info->D[1];
//       parameters.p1 = camera_info->D[2];
//       parameters.p2 = camera_info->D[3];
//       parameters.k3 = camera_info->D[4];
//       return;
//     }
//   }

//   bool got_cam_calibration = true;
//   if (sensor_== ORB_SLAM3::System::STEREO || sensor_==ORB_SLAM3::System::RGBD) {
//     got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_baseline", parameters.baseline);
//   }

//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p1", parameters.p1);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p2", parameters.p2);
//   got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k3", parameters.k3);

//   if (!got_cam_calibration) {
//     ROS_ERROR ("Failed to get camera calibration parameters from the launch file.");
//     throw std::runtime_error("No cam calibration");
//   }
// }
}