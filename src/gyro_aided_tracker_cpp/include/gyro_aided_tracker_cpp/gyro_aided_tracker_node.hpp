#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <chrono>

//custom message to publish the pose health
#include "gyro_aided_tracker_cpp/msg/pose_health.hpp"
#include <cmath>


//struct is used for data, which will have bag of values to pass around. 
// std::deque<IMUSample> will fill with these values.
struct IMUSample
{
    int64_t timestep_ns;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;

};


struct TrackResult
{
    int n_features; //Track how many feature are alive.
    int n_lost;   // track how many feature died in this frame.
    int n_new;    // track how many feature are new.
    float median_prediction_error; //gyro prediction vs KLT actual (px)
};


class GyroAidedTracker
{
public:
    GyroAidedTracker( cv::Mat K, int max_features, int min_features,
                        int grid_size, int klt_win_size, float ransac_thresh );
    
    TrackResult track(const cv::Mat& prev_gray,
                        const cv::Mat& curr_gray,
                    const std::vector<IMUSample>& imu_samples);
private:

    cv::Mat K_ ; //3*3 intrinsic matrix
    std::vector<cv::Point2f> features_; //currently tracked feature points
    int max_features_, min_features_;
    int grid_size_, klt_win_size_;
    float ransac_thresh_;



    Eigen::Matrix3d integrateGyro(const std::vector<IMUSample>& samples);
    void detectFeatures(const cv::Mat& gray);
    std::vector<cv::Point2f> predictFeatures(const Eigen::Matrix3d& R);




};

class TrackerNode : public rclcpp::Node
{
public: 
    TrackerNode();

private:
    //owns the tracker
    GyroAidedTracker tracker_;

    //IMU buffer, shared between callbacks, needs a lock
    std::deque<IMUSample> imu_buffer_;
    std::mutex imu_mutex_;

    //frame state

    cv::Mat prev_gray_;
    int64_t t_prev_ns_= 0;

    //latest health - written by image callback, read by odom callback
    TrackResult latest_result_;
    int tracking_state_;



    //ROS_handles
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // posehealth publisher
    rclcpp::Publisher<gyro_aided_tracker_cpp::msg::PoseHealth>::SharedPtr pub_health_;

    //callbacks

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void diagnosticsTimer();

};

