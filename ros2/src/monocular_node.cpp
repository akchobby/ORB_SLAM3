#include "monocular_node.hpp"


MonocularNode::MonocularNode(): orb_slam3_ros::Node(ORB_SLAM3::System::MONOCULAR, "orbslam_monocular"){

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera",
        10,
        std::bind(&MonocularNode::image_callback, this, std::placeholders::_1));

}

MonoNode::~MonoNode () {

}

void MonocularNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    current_frame_time_ = msg->header.stamp;
    std::cout<<"a frame has been sent"<<std::endl;
    orb_slam_->TrackMonocular(m_cvImPtr->image, stamp_to_sec(msg->header.stamp));

}