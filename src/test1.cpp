#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat current_image;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        current_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cv::Mat resized_image;

        // 检查图像是否为空
        if (!current_image.empty()) {
            // 设置缩放后的目标大小
            int target_width = 1280; // 可以根据需要调整
            int target_height = 1024; // 可以根据需要调整

            // 计算缩放比例
            double scale = std::min(target_width / (double)current_image.cols, target_height / (double)current_image.rows);

            // 缩放图像
            cv::resize(current_image, resized_image, cv::Size(), scale, scale, cv::INTER_LINEAR);

            cv::imshow("Image Window", resized_image);
            int key = cv::waitKey(30);
            if (key == ' ') {  // 按空格键保存图像
                // 保存原始尺寸的图像
                std::string filename = "/home/quintinumi/project/hikcamera_cal_img/camera_1/saved_image_" + std::to_string(ros::Time::now().toSec()) + ".png";
                cv::imwrite(filename, current_image);
                ROS_INFO("Image saved as %s", filename.c_str());
            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // 创建一个可调整大小的窗口
    cv::namedWindow("Image Window", cv::WINDOW_NORMAL);
    cv::resizeWindow("Image Window", 1280, 1024);  // 设置初始窗口大小

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/hikcamera/image_1", 1, imageCallback);

    ros::spin();

    cv::destroyWindow("Image Window");
    return 0;
}