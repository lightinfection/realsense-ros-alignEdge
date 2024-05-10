#pragma once

#include <math.h>

#include <iostream>
#include <vector>
#include <memory>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"

struct depth2pc
{
    float x, y, z;
};

class edge_detection : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aligned_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;

    sensor_msgs::msg::Image::SharedPtr output_img, aligned_depth_rgb;
    sensor_msgs::msg::PointCloud2::UniquePtr border_pc;
    rclcpp::SensorDataQoS sensor_qos;

    bool use_camera_info_, apply_blur_post_, apply_blur_pre_, L2gradient_, apply_erode_;
    int edge_type_, queue_size_, canny_threshold1_, canny_threshold2_, apertureSize_, postBlurSize_, iterations_, frame_height_, frame_bottom_;
    float postBlurSigma_, min_range_, max_range_, min_height_, max_height_;
    std::string frame_id;
    enum edge_type
    {
        Sobel_Derivatives = 0,
        Laplace_Operator = 1,
        Canny_Edge_Detector = 2
    };

    float center_x, center_y, constant_x, constant_y;
    const double unit_scaling = (uint16_t)1 * 0.001f;
    depth2pc edge_point;
    
    std::vector<sensor_msgs::msg::PointField> fields;
    std::vector<depth2pc> tar;

public:
    edge_detection();
    ~edge_detection() override;    

    bool info_found = false, depth_found = false, start = false;
    void img_process(const sensor_msgs::msg::Image::Ptr OriginalImg);
    void get_info(const sensor_msgs::msg::CameraInfo::Ptr info_);
    void get_depth(const sensor_msgs::msg::Image::Ptr depth_);

    cv::Mat cannyEdgeDetection(const sensor_msgs::msg::Image::Ptr msg);
    void border2pc(const cv::Mat& border, const sensor_msgs::msg::Image::Ptr depth_);
    bool publish_pc(const std::vector<depth2pc>& tar_);
    cv::Mat publish_img(const cv::Mat& border);
};
