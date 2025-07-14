// Code from https://github.com/TIERS/dynamic_scan_tracking/tree/main
// That was adapted from https://github.com/udacity/CarND-Extended-Kalman-Filter-Project
// And corrected by us based on this: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9732665

#pragma once

#include "Eigen/Dense"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <visualization_msgs/Marker.h>

enum KFMotionModelType {
  KFConstVelocityDogru = 0,
  KFConstantTurnRateCatalano = 1,
  KFConstantTurnRate = 2,
  KFConstantTurnRateWithZVelocity = 3,
};

class MAVTrackKF {
  private:
    // first measurement flag
    bool first_measurement;

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    // process noise variable for Kalman filter Q
    float q_noise;

    std::string frame;

    double search_radius;

    KFMotionModelType motion_model_type;

    bool dogru_use_min_search_radius;

  public:
    /**
     * Constructor
     */
    MAVTrackKF();

    /**
     * Destructor
     */
    virtual ~MAVTrackKF();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void init(
        Eigen::VectorXd &x_in,
        Eigen::MatrixXd &P_in,
        Eigen::MatrixXd &F_in,
        Eigen::MatrixXd &H_in,
        Eigen::MatrixXd &R_in,
        Eigen::MatrixXd &Q_in,
        double search_radius,
        KFMotionModelType motion_model_type,
        std::string frame = "world",
        bool dogru_use_min_search_radius = false);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void update(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * Predicts the state by using Extended Kalman Filter equations
     * @param delta_T Time between k and k+1 in s
     */
    void predictEKF(float delta_T);

    void getCloudCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::VectorXd &z);

    bool is_first_measurement();

    void set_first_measurement(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    void check_with_KDTree(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

    bool get_is_tracked();
    
    visualization_msgs::Marker getEstimate();
};