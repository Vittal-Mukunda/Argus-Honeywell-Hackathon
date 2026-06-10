/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>   // ARGUS C2: SuperPoint keypoints
#include <algorithm>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::msg::Imu::ConstPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstPtr> feature_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img0_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img1_buf;
std::mutex m_buf;

// ===================== ARGUS C2: SuperPoint front-end =======================
// Buffer learned keypoints (/argus/vio/keypoints) by stamp; just before each
// inputImage() we hand the matching frame's detections to the feature tracker,
// which uses them in place of goodFeaturesToTrack. Only active when the config's
// use_superpoint:1 (USE_SUPERPOINT) is set; C1 baseline ignores all of this.
std::mutex m_kp;
queue<sensor_msgs::msg::PointCloud2::ConstPtr> kp_buf;
long g_sp_match = 0, g_sp_fallback = 0;

void kp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    m_kp.lock();
    kp_buf.push(msg);
    while (kp_buf.size() > 90)   // bound memory (~6 s @ 15 Hz)
        kp_buf.pop();
    m_kp.unlock();
}

// Decode PointCloud2 (x=u, y=v, z=score; 12-byte point_step from superpoint_node)
// into image points sorted by score DESC, so the tracker prefers confident kpts.
static void cloudToKpts(const sensor_msgs::msg::PointCloud2::ConstPtr &c,
                        std::vector<cv::Point2f> &out)
{
    out.clear();
    const size_t n = static_cast<size_t>(c->width) * c->height;
    if (n == 0 || c->point_step < 12 || c->data.size() < n * c->point_step)
        return;
    std::vector<std::pair<float, cv::Point2f>> tmp;
    tmp.reserve(n);
    const uint8_t *base = c->data.data();
    for (size_t i = 0; i < n; ++i)
    {
        const float *f = reinterpret_cast<const float *>(base + i * c->point_step);
        tmp.emplace_back(f[2], cv::Point2f(f[0], f[1]));
    }
    std::sort(tmp.begin(), tmp.end(),
              [](const std::pair<float, cv::Point2f> &a,
                 const std::pair<float, cv::Point2f> &b) { return a.first > b.first; });
    out.reserve(tmp.size());
    for (auto &p : tmp)
        out.push_back(p.second);
}

// Pop clouds older than t; if the front matches t (+/-tol) decode it into out.
static bool getKeypointsForStamp(double t, std::vector<cv::Point2f> &out)
{
    const double tol = 0.002;   // images + keypoints carry the exact same cam0 stamp
    bool found = false;
    m_kp.lock();
    while (!kp_buf.empty())
    {
        double kt = kp_buf.front()->header.stamp.sec +
                    kp_buf.front()->header.stamp.nanosec * 1e-9;
        if (kt < t - tol) { kp_buf.pop(); continue; }   // stale -> drop
        if (kt <= t + tol)                              // matched this frame
        {
            cloudToKpts(kp_buf.front(), out);
            kp_buf.pop();
            found = true;
        }
        break;   // front is >= t-tol: matched, or it belongs to a future frame
    }
    m_kp.unlock();
    return found;
}

// Look up (with bounded wait, since the async SuperPoint node may lag image
// intake), set the detections on the tracker, and tally match/fallback.
static void feedSuperpoint(double time)
{
    std::vector<cv::Point2f> kpts;
    int tries = 0;
    while (!getKeypointsForStamp(time, kpts) && tries++ < 250)   // <= ~0.5 s wall
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!kpts.empty())
    {
        estimator.featureTracker.setExternalKeypoints(kpts);
        ++g_sp_match;
    }
    else
        ++g_sp_fallback;   // tracker falls back to goodFeaturesToTrack this frame
    if (((g_sp_match + g_sp_fallback) % 50) == 0)
        printf("[C2] superpoint frames matched=%ld fallback=%ld\n",
               g_sp_match, g_sp_fallback);
}
// ============================================================================

// header: 1403715278
void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Left : " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Right: " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img1_buf.push(img_msg);
    m_buf.unlock();
}


// cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr img_msg)
cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * (1e-9);

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    // Drop old buffered frames to keep up with real-time
                    while(img0_buf.size() > 1 && img1_buf.size() > 1)
                    {
                        img0_buf.pop();
                        img1_buf.pop();
                    }
                }
            }
            m_buf.unlock();
            if(!image0.empty())
            {
                if (USE_SUPERPOINT)
                    feedSuperpoint(time);   // ARGUS C2: hand learned kpts to tracker
                estimator.inputImage(time, image0, image1);
            }
        }
        else
        {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                // Drop old buffered frames to keep up with real-time
                while(img0_buf.size() > 1)
                {
                    img0_buf.pop();
                }
            }
            m_buf.unlock();
            if(!image.empty())
            {
                if (USE_SUPERPOINT)
                    feedSuperpoint(time);   // ARGUS C2: hand learned kpts to tracker
                estimator.inputImage(time, image);
            }
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // std::cout << "IMU cb" << std::endl;

    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);

    // std::cout << "got t_imu: " << std::fixed << t << endl;
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    std::cout << "feature cb" << std::endl;
    std::cout << "Feature: " << feature_msg->points.size() << std::endl;


    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * (1e-9);
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("vins_estimator");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    if(non_ros_args.size() != 2)
    {
        printf("please intput: ros2 run vins vins_node [config file] \n"
               "for example: ros2 run vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = non_ros_args[1];
    printf("config_file: %s\n", config_file.c_str());

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu = NULL;
    if(USE_IMU)
    {
        // ARGUS patch: RELIABLE to match the gz bridge / rosbag2 player (both
        // publish RELIABLE). A BEST_EFFORT sub receives nothing from the player
        // over Cyclone despite "compatible" QoS (known VINS-ROS2 + bag quirk).
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)).reliable(), imu_callback);
    }
    auto sub_feature = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_img0 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(), img0_callback);

    // ARGUS C2: subscribe SuperPoint keypoints only when the front-end is enabled.
    // RELIABLE to byte-match the publisher (superpoint_node) over Cyclone (dev #7).
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_kp = NULL;
    if (USE_SUPERPOINT)
    {
        sub_kp = n->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/argus/vio/keypoints", rclcpp::QoS(rclcpp::KeepLast(100)).reliable(), kp_callback);
        printf("[C2] SuperPoint front-end ENABLED: subscribing /argus/vio/keypoints\n");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1 = NULL;
    if(STEREO)
    {
        sub_img1 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(), img1_callback);
    }

    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)), restart_callback);
    auto sub_imu_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_imu_switch", rclcpp::QoS(rclcpp::KeepLast(100)), imu_switch_callback);
    auto sub_cam_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_cam_switch", rclcpp::QoS(rclcpp::KeepLast(100)), cam_switch_callback);

    std::thread sync_thread{sync_process};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(n);
    executor.spin();

    return 0;
}
