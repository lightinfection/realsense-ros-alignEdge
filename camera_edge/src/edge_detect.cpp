#include "camera_edge/edge_detect.hpp"

edge_detection::edge_detection()
: rclcpp::Node("edge_detect")
{
    edge_type_ = this->declare_parameter("edge_type", 0);
    canny_threshold1_ = this->declare_parameter("canny_threshold1", 100);
    canny_threshold2_ = this->declare_parameter("canny_threshold2", 200);
    apertureSize_ = this->declare_parameter("apertureSize", 3);
    apply_blur_pre_ = this->declare_parameter("apply_blur_pre", true);
    postBlurSize_ = this->declare_parameter("postBlurSize", 3);
    postBlurSigma_ = this->declare_parameter("postBlurSigma", 3.2);
    apply_blur_post_ = this->declare_parameter("apply_blur_post", false);
    L2gradient_ = this->declare_parameter("L2gradient", false);
    iterations_ = this->declare_parameter("morph_iteration", 1);
    apply_erode_ = this->declare_parameter("apply_erode", false);

    frame_height_ = this->declare_parameter("frame_height", 1);
    frame_bottom_= this->declare_parameter("frame_bottom", 720);
    min_range_ = this->declare_parameter("min_range", 0.0);
    max_range_ = this->declare_parameter("max_range", 10.0);
    min_height_ = this->declare_parameter("min_height", -2.0);
    max_height_ = this->declare_parameter("max_height", 2.0);
    frame_id = this->declare_parameter("frame_id"," camera_link");

    output_img = std::make_shared<sensor_msgs::msg::Image>();
    border_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();

    aligned_img_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_rect_raw", sensor_qos, std::bind(&edge_detection::get_depth, this, std::placeholders::_1));
    info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", sensor_qos, std::bind(&edge_detection::get_info, this, std::placeholders::_1));
    img_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_rect_raw", sensor_qos, std::bind(&edge_detection::img_process, this, std::placeholders::_1));
    
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_edge", sensor_qos);
    pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/border/points", sensor_qos);
}

edge_detection::~edge_detection()
{
    RCLCPP_INFO(this->get_logger(), "Closing edge detection...");
}

void edge_detection::img_process(const sensor_msgs::msg::Image::Ptr OriginalImg)
{
    output_img->header = OriginalImg->header;
    if(OriginalImg->encoding == "8UC1") output_img->encoding = "mono8";
    if(OriginalImg->encoding == "rgb8") output_img->encoding = "mono8";
    if(OriginalImg->encoding == "16UC1") output_img->encoding = "mono16";
    cv::Mat border = cannyEdgeDetection(OriginalImg);
    if (info_found && depth_found)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "start");
        border2pc(border, aligned_depth_rgb);
    }
}

cv::Mat edge_detection::cannyEdgeDetection(const sensor_msgs::msg::Image::Ptr msg)
{
    // Work on the image.
    try
    {
        // Convert the image into something opencv can handle.
        cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;
        if (apply_blur_pre_) cv::GaussianBlur(frame, frame, cv::Size(apertureSize_, apertureSize_), 0, 0, cv::BORDER_DEFAULT);
        
        // Convert it to gray
        cv::Mat src_gray;
        if (frame.channels() > 1)
        {
            cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
        }
        else
        {
            src_gray = frame;
        }

        cv::Mat grad;
        switch (edge_type_)
        {
            case edge_type::Sobel_Derivatives:
            {
            /// Generate grad_x and grad_y
            cv::Mat grad_x, grad_y;
            cv::Mat abs_grad_x, abs_grad_y;

            int scale = 1;
            int delta = 0;
            int ddepth = CV_16S;

            cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
            cv::convertScaleAbs(grad_x, abs_grad_x);
            cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
            cv::convertScaleAbs(grad_y, abs_grad_y);
            cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
            break;
            }

            case edge_type::Laplace_Operator:
            {
            cv::Mat dst;
            int kernel_size = 3;
            int scale = 1;
            int delta = 0;
            int ddepth = CV_16S;
            cv::Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
            convertScaleAbs(dst, grad);
            break;
            }

            case edge_type::Canny_Edge_Detector:
            {
            int kernel_size = 3;
            cv::Mat detected_edges;
            cv::Canny(src_gray, grad, canny_threshold1_, canny_threshold2_, kernel_size, L2gradient_);
            break;
            }
        }
        if (apply_blur_post_) cv::GaussianBlur(grad, grad, cv::Size(postBlurSize_, postBlurSize_), postBlurSigma_, postBlurSigma_, cv::BORDER_DEFAULT);
        return grad;
    }
    catch (cv::Exception& e)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(),"Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    // prev_stamp_ = msg->header.stamp;
}

void edge_detection::get_info(const sensor_msgs::msg::CameraInfo::Ptr info_)
{
    if(!info_found)
    {
        image_geometry::PinholeCameraModel cam_info_;
        cam_info_.fromCameraInfo(info_);
        center_x = cam_info_.cx();
        center_y = cam_info_.cy();
        constant_x = unit_scaling / cam_info_.fx();
        constant_y = unit_scaling / cam_info_.fy();
        info_found = true;
    }
}

void edge_detection::get_depth(const sensor_msgs::msg::Image::Ptr depth_)
{
    if (!start)
    {
        aligned_depth_rgb = depth_;
        depth_found = true;
    }
}

void edge_detection::border2pc(const cv::Mat& border, const sensor_msgs::msg::Image::Ptr depth_)
{
    start = true;
    if (border.channels() != 1)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Border Extraction Error");
        start = false;
        depth_found = false;
        return;
    }
    cv::Mat final = publish_img(border);
    if (final.size().width!=(int)(depth_->width) || final.size().height!=(int)(depth_->height)) 
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "rgb and depth images are not aligned");
        start = false;
        depth_found = false;
        return;
    }
    if (frame_height_ > final.size().height || frame_bottom_ > final.size().height)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "frame_height or frame_bottom is greater than the maxium height of the depth image");        
        start = false;
        depth_found = false;
        return;
    }
    try
    {
        const uint16_t* p = reinterpret_cast<const uint16_t*>(&depth_->data[0]);
        int row_step = depth_->step / sizeof(uint16_t);
        p += row_step * (frame_height_-1);        
        for (int i = frame_height_-1; i < frame_bottom_ ; i++, p += row_step)
        {
            for (uint32_t j = 0; j < depth_->width; j++)
            {
                if (final.at<uint8_t>(i,j)!=0)
                {
                    uint16_t depth = p[j];
                    edge_point.x = depth * 0.001f;
                    if (edge_point.x >= min_range_ && edge_point.x <= max_range_)
                    {
                        edge_point.z = -(i - center_y) * depth * constant_y;
                        if (edge_point.z >= min_height_ && edge_point.z <= max_height_)
                        {
                            edge_point.y = -(j - center_x) * depth * constant_x;
                            tar.push_back(edge_point);
                        }
                    }
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), e.what());
        start = false;
        depth_found = false;
        return;
    }
    publish_pc(tar);
    tar.clear();
    start = false;
    depth_found = false;
}

cv::Mat edge_detection::publish_img(const cv::Mat& border)
{
    try
    {
        cv::Mat thresh, output;
        cv::threshold(border, thresh, 0, 255, cv::THRESH_TRIANGLE);
        if (iterations_>0) cv::morphologyEx(thresh, output, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), iterations_);
        else thresh.copyTo(output);
        if (apply_erode_) cv::morphologyEx(output, output, cv::MORPH_ERODE, cv::Mat(), cv::Point(-1,-1), 1);
        
        output_img = cv_bridge::CvImage(output_img->header, output_img->encoding, output).toImageMsg();
        img_pub->publish(*output_img);
        return output;
    }
    catch (cv::Exception& e)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(),"Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
}

bool edge_detection::publish_pc(const std::vector<depth2pc>& tar_)
{
    try
    {
        border_pc->data.clear();
        if (fields.empty())
        {    
            uint8_t offset = 0;
            border_pc->header.frame_id = frame_id;
            border_pc->height = 1;
            border_pc->is_bigendian = false;
            border_pc->is_dense = false;
            fields.resize(3);
            fields[0].name = "x"; 
            fields[1].name = "y"; 
            fields[2].name = "z"; 
            for (size_t i = 0; i < fields.size(); i++, offset+=4)
            {
                fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
                fields[i].offset = offset;
                fields[i].count = 1;
            }
            border_pc->point_step = offset;
        }
        border_pc->header.stamp = output_img->header.stamp;
        border_pc->width = tar_.size();
        border_pc->row_step = border_pc->point_step*border_pc->width;
        border_pc->fields = fields;
        border_pc->data.resize(border_pc->height*border_pc->row_step);
        for (size_t pc = 0; pc < tar_.size(); pc++)
        {
            memcpy(
                &border_pc->data[pc*border_pc->point_step + border_pc->fields[0].offset], 
                &tar_[pc].x, sizeof(float));
            memcpy(
                &border_pc->data[pc*border_pc->point_step + border_pc->fields[1].offset], 
                &tar_[pc].y, sizeof(float));
            memcpy(
                &border_pc->data[pc*border_pc->point_step + border_pc->fields[2].offset], 
                &tar_[pc].z, sizeof(float));
        }
        pc_pub->publish(*border_pc);
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), e.what());
        return false;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<edge_detection> nh = std::make_shared<edge_detection>();
    rclcpp::spin(nh);
    rclcpp::shutdown();
}