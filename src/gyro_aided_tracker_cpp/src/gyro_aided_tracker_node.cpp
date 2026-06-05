#include <gyro_aided_tracker_cpp/gyro_aided_tracker_node.hpp>
// #include "gyro_aided_tracker_cpp/msg/pose_health.hpp"

using namespace std;

GyroAidedTracker::GyroAidedTracker( cv::Mat K, int max_features, int min_features,
                        int grid_size, int klt_win_size, float ransac_thresh )
                        : K_(K.clone()),
                            max_features_(max_features),
                            min_features_(min_features),
                            grid_size_(grid_size),
                            klt_win_size_(klt_win_size),
                            ransac_thresh_(ransac_thresh)
{

}

Eigen::Matrix3d GyroAidedTracker::integrateGyro(const std::vector<IMUSample>& samples)
{
    Eigen::Matrix3d R_total = Eigen::Matrix3d::Identity();

    for(size_t i=1; i < samples.size();i++)
    {
        double dt = (samples[i].timestep_ns - samples[i-1].timestep_ns) * 1e-9;

        Eigen::Vector3d omega = samples[i].gyro; //rad/s

        double theta = omega.norm() * dt;   // rotation angle (rad)

        if (theta < 1e-9) continue;

        Eigen::Vector3d axis = omega / omega.norm(); //omega.norm() will the give the magnitude sqrt(wx2 + wy2 + wz2)

        //skew-symmetric matrix

        Eigen::Matrix3d k;

        k << 0, -axis.z(),  axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0;

        // using the Rodrigues Formula

        Eigen::Matrix3d dR = Eigen::Matrix3d::Identity() + 
                            std::sin(theta) * k + (1 - std::cos(theta)) * k * k;


        R_total = dR * R_total;

    }
    return R_total; // projects each tracked feature point through it to get a predicted position for the next frame;



}


void GyroAidedTracker::detectFeatures(const cv::Mat& gray)
{
    features_.clear();
    int cell_w = gray.cols / grid_size_;
    int cell_h = gray.rows /grid_size_;

    for(int row = 0; row < grid_size_ && (int)features_.size() < max_features_; row++ )
    {
        for(int col=0; col < grid_size_ && (int)features_.size() < max_features_; col++)
        {   
            //define this cell's region
            cv::Rect cell(col * cell_w, row * cell_h, cell_w, cell_h );
            cv::Mat roi = gray(cell);

            //detect corners in the cell
            std::vector<cv::Point2f> cell_pts;
            cv::goodFeaturesToTrack(roi, cell_pts,
                 5, // max per cell
                 0.01, // quality level
                 5.0 // min distance between features
                ); 
            
            // offset back to full image coordinated
            for (auto& pt : cell_pts)
            {
                pt.x += col*cell_w;
                pt.y += row*cell_h;
                features_.push_back(pt);
            }

        }
    }
}

std::vector<cv::Point2f> GyroAidedTracker::predictFeatures(const Eigen::Matrix3d& R)
{
    //build k as Eigen matrix for math

    Eigen::Matrix3d k_e;
/** 
    K = [ fx   0   cx ]
    [  0  fy   cy ]       //fx, fy — focal lengths in pixels
    [  0   0    1 ]       // cx, cy — principal point (optical center, usually image center)

*/

    k_e << K_.at<double>(0,0), 0,                   K_.at<double>(0,2),
            0,                 K_.at<double>(1,1),  K_.at<double>(1,2),
            0,                 0,                   1.0; 

    Eigen::Matrix3d k_inv = k_e.inverse();
    Eigen::Matrix3d H = k_e * R * k_inv; //full homography - project rotation onto the image plane

    std::vector<cv::Point2f> predicted;

    predicted.reserve(features_.size());  // initializing the space for prediction - Pre-allocates memory — avoids repeated reallocation in the loop

    for(const auto& pt : features_)
    {
        Eigen::Vector3d p(pt.x, pt.y,1.0); //homogeneous point
        Eigen::Vector3d p_pred = H * p; // apply homography

        predicted.push_back(cv::Point2f(static_cast<float>(p_pred.x()/p_pred.z()),  // p_pred.z() divides to normalize from homogeneous back to 2D
                                        static_cast<float>(p_pred.y()/p_pred.z())
                                    ));
        
    }
    return predicted;


    
}

TrackResult GyroAidedTracker::track(const cv::Mat& prev_gray,
                        const cv::Mat& curr_gray,
                    const std::vector<IMUSample>& imu_samples)
{
    //step1: bootstrap if no features

    if (features_.empty()){
        detectFeatures(prev_gray);
    }
    int n_before = static_cast<int>(features_.size());

    // step 2 and 3 : gyro_pridiction

    Eigen::Matrix3d R = integrateGyro(imu_samples);

    std::vector<cv::Point2f> predicted = predictFeatures(R);

    // step 4: KLT optical flow
    std::vector<cv::Point2f> tracked_pts;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_gray,curr_gray,
        predicted, //initial guess from gyro
        tracked_pts,
        status,
        err,
        cv::Size(klt_win_size_ * 2 + 1, klt_win_size_ * 2 + 1),
        3   //pyramid level
    
    );
    //step 5: filter by klt status

    std::vector<cv::Point2f> prev_good, curr_good;
    std::vector<float> pred_errors;

    for(size_t i=0; i < status.size(); i++)
    {
        if(!status[i]) continue;

        //pridiction error  = distance betweeen gyro guess and KLT result
        float dx = tracked_pts[i].x - predicted[i].x;
        float dy = tracked_pts[i].y - predicted[i].y;

        pred_errors.push_back(std::sqrt(dx*dx + dy*dy));
        
        prev_good.push_back(features_[i]);
        curr_good.push_back(tracked_pts[i]);

    }

    //step 6: RANSAC outlier Rejection

    if(prev_good.size()>= 4){
        std::vector<uchar> inlier_mask;
        cv::findHomography(prev_good, curr_good, cv::RANSAC, ransac_thresh_, inlier_mask);

        std::vector<cv::Point2f> inlier_pts;
        for(size_t i=0; i < inlier_mask.size(); i++)
        {
            if(inlier_mask[i])
                inlier_pts.push_back(curr_good[i]);

        }
        features_ = inlier_pts;
    }
    else
    {
        features_ = curr_good;  // too few for RANSAC, keep what we have
    }

    //step 7: median prediction error

    float median_err = 0.0f;
    if(!pred_errors.empty()){
        std::sort(pred_errors.begin(), pred_errors.end());
        median_err = pred_errors[pred_errors.size()/2];
    }

    //step 8: replenish if below threshold

    int n_lost = n_before - static_cast<int>(features_.size());
    int n_before_replenish = static_cast<int>(features_.size());

    if((int)features_.size()<min_features_)
        detectFeatures(curr_gray);

    int n_new = static_cast<int>(features_.size()) - n_before_replenish;

    // return health

    TrackResult result;

    result.n_features = static_cast<int>(features_.size());
    result.median_prediction_error = median_err;
    result.n_lost = std::max(0,n_lost);
    result.n_new = std::max(0,n_new);

    return result;

}

static cv::Mat build(double fx, double fy, double cx, double cy)
{
    cv::Mat K = cv::Mat::eye(3,3, CV_64F);
    K.at<double>(0,0) = fx;
    K.at<double>(1,1) = fy;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;
    return K;

}
TrackerNode::TrackerNode() : rclcpp::Node("gyro_aided_tracker"), tracker_(build(527.0, 527.0, 640.0, 360.0), 200, 50, 8, 7, 1.5f)
{
    
        //QoS: sensor data = best effort, keep last 1
        auto sensor_qos = rclcpp::QoS(1).best_effort();

        //subscribers
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/zed2/zed_node/imu/data",sensor_qos,
                        std::bind(&TrackerNode::imuCallback,this,std::placeholders::_1));
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>("/zed2/zed_node/left/image_rect_gray",sensor_qos,
                        std::bind(&TrackerNode::imageCallback,this,std::placeholders::_1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/zed2/zed_node/odom", sensor_qos,
                        std::bind(&TrackerNode::odomCallback,this,std::placeholders::_1));
        //Publisher

        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov",10);
        pub_health_ = this->create_publisher<gyro_aided_tracker_cpp::msg::PoseHealth>("/pose_health",10);

        //Diagnostic timer at 1 HZ

        diag_timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&TrackerNode::diagnosticsTimer,this));

        RCLCPP_INFO(this->get_logger(),"Gyro-aided tracker node started");
}
void TrackerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    IMUSample sample;

    sample.timestep_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000
                        + msg->header.stamp.nanosec;

    sample.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                                    msg->angular_velocity.y,
                                    msg->angular_velocity.z);
    sample.accel = Eigen::Vector3d(msg->linear_acceleration.x,
                                    msg->linear_acceleration.y,
                                    msg->linear_acceleration.z);

    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(sample);

}

void TrackerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    int64_t t_curr_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000
                            + msg->header.stamp.nanosec;
    
    // convert the ROS image to Opencv grayscale

    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,"mono8");
    }catch (const cv_bridge::Exception& e)
    {
        RCLCPP_WARN(this->get_logger(),"cv_bridge failed: %s", e.what());
        return;
    }
    cv::Mat curr_gray = cv_ptr->image;

    // sweep IMU buffer (lock held briefly)

    std::vector<IMUSample> samples;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        for(const auto& s : imu_buffer_)
            if(s.timestep_ns > t_prev_ns_ && s.timestep_ns <= t_curr_ns)
                samples.push_back(s);
    }
    //lock realeased here, tracker.track() runs outside lock

    // first frame bootstrap

    if(prev_gray_.empty())
    {
        prev_gray_ = curr_gray.clone();
        t_prev_ns_ = t_curr_ns;
        return;
    }

    //run tracker

    TrackResult result = tracker_.track(prev_gray_,curr_gray,samples);

    //assess health
    if(result.n_features>=80)
    {
        if(result.median_prediction_error < 5.0f)
            tracking_state_ = 0; // Healthy
        else{
            tracking_state_ = 1; // Marginal
        }
    }
    else if(result.n_features >= 30)
    {
        tracking_state_ = 1; // MARGINAL
    }
    else if(result.n_features > 5)
    {
        tracking_state_ = 2; //degraded
    }
    else{
        tracking_state_ = 3; //lost
    }

    latest_result_ = result;

    float cov_scale;
    // compute cov_scale from state
    switch (tracking_state_)
    {
    case 0: cov_scale = 1.0f; break;
    case 1: cov_scale = 3.0f; break;
    case 2: cov_scale = 10.0f; break;
    
    default: cov_scale = -1.0f; break;
    }

    //publish health
    gyro_aided_tracker_cpp::msg::PoseHealth health_msg;
    health_msg.header = msg->header;
    health_msg.n_features = result.n_features;
    health_msg.n_lost = result.n_lost;
    health_msg.n_new  = result.n_new;
    health_msg.median_pred_error_px = result.median_prediction_error;
    health_msg.tracking_state = static_cast<uint8_t>(tracking_state_);
    health_msg.covariance_scale = cov_scale;

    //gyro/accel magnitude from last IMU sample
    if(!samples.empty())
    {
        health_msg.gyro_magnitude_dps = static_cast<float>(
            samples.back().gyro.norm() * 180 / M_PI);
        health_msg.accel_magnitude = static_cast<float>(
            samples.back().accel.norm()
        );
    }
    pub_health_->publish(health_msg);

    //update frame state
    prev_gray_ = curr_gray.clone();
    t_prev_ns_ = t_curr_ns;



}

void TrackerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  //compute covariance scale from tracking state
    float cov_scale = 1.0f;

    switch(tracking_state_)
    {
        case 0: 
            cov_scale = 1.0f;
            break;
        case 1:
            cov_scale = 3.0f;
            break;
        case 2: 
            cov_scale = 10.0f;
            break;
        case 3:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                    "Pose Suppressed: Tracking Lost(%d features)",
                                latest_result_.n_features);
            return;
        default:
            cov_scale = 5.0f;
            break;
    }

    // Build PoseWithCovarianceStamped

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.header.frame_id = "odom";
    out.pose.pose = msg->pose.pose;

    //copy covariance and scale position diagonal(xx, yy, zz)
    out.pose.covariance = msg->pose.covariance;
    out.pose.covariance[0] *= cov_scale; // xx
    out.pose.covariance[7] *= cov_scale; // yy
    out.pose.covariance[14] *= cov_scale; // zz

    pub_pose_->publish(out);
}

void TrackerNode::diagnosticsTimer()
{
    if(prev_gray_.empty())
    {
        RCLCPP_INFO(this->get_logger(),"wating for the first frame....");
        return;
    }
    const char* state_names[] = {"Healthy", "Marginal", "Degraded", "Lost"};
    //below is ternary operatory if else in one line
    const char* state_str = (tracking_state_>=0 && tracking_state_<=3)
                            ? state_names[tracking_state_]
                            : "UNKNOWN";
    RCLCPP_INFO(this->get_logger(),
                    "[%s] features=%d lost=%d new=%d pred_err=%.2fpx",
                state_str,
                latest_result_.n_features,
                latest_result_.n_lost,
                latest_result_.n_new,
                latest_result_.median_prediction_error);
}





int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<TrackerNode>();

    //MultithreadedExecuter so IMU(400Hz) and image(60Hz)
    // callbacks can run concurrently, this is why imu_mutex_ matters

    rclcpp::executors::MultiThreadedExecutor executer(
        rclcpp::ExecutorOptions(),3
    ); //3 threads
    executer.add_node(node);
    executer.spin();

    rclcpp::shutdown();
    return 0;
}