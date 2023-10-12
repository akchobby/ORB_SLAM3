#pragma once

#include "node.hpp"

class MonocularNode : public orb_slam3_ros::Node {

    public:
        MonocularNode();
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    private:
      
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> image_subscriber_;

};