// Code from https://github.com/TIERS/dynamic_scan_tracking/tree/main
// That was adapted from https://github.com/udacity/CarND-Extended-Kalman-Filter-Project
// And corrected by us based on this: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9732665

#include "mav_track/mav_track_KF.hpp"

#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>

// Tracy
#include "tracy/Tracy.hpp"

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

UAVTrackKF::UAVTrackKF() {}

UAVTrackKF::~UAVTrackKF() {}

void UAVTrackKF::init(
        Eigen::VectorXd &x_in,
        Eigen:: MatrixXd &P_in,
        Eigen::MatrixXd &F_in,
        Eigen::MatrixXd &H_in,
        Eigen::MatrixXd &R_in,
        Eigen::MatrixXd &Q_in,
        double search_radius,
        KFMotionModelType motion_model_type,
        std::string frame,
        bool dogru_use_min_search_radius) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
    this->search_radius = search_radius;
    this->motion_model_type = motion_model_type;
    this->frame = frame;
    this->first_measurement = true;
    this->dogru_use_min_search_radius = dogru_use_min_search_radius;
}

void UAVTrackKF::predictEKF(float delta_T) {
    ZoneScoped;
    // input state values (not all of them are used in all models)
    double x, y, z, v_x, v_y, v_z, v, yaw, yawd;
    //predicted state values (not all of them are used in all models)
    double x_p, y_p, z_p, v_x_p, v_y_p, v_z_p, v_p, yaw_p, yawd_p;

    switch (motion_model_type) {
        case KFConstVelocityDogru:
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v_x = x_(3);
            v_y = x_(4);
            v_z = x_(5);

            x_p = x + v_x * delta_T;
            y_p = y + v_y * delta_T;
            z_p = z + v_z * delta_T;
            v_x_p = v_x;
            v_y_p = v_y;
            v_z_p = v_z;

            // Calculate the Jacobian of the Transition Matrix F for CTRV model
            F_.setIdentity();
            F_(0,3) = delta_T;
            F_(1,4) = delta_T;
            F_(2,5) = delta_T;
    
            // CORRECTION: had to update the current state with the predicitons
            x_ << x_p, y_p, z_p, v_x, v_y, v_z;
            
            P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix

            break;
        case KFConstantTurnRateCatalano:
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v = x_(3);
            yaw = x_(4);
            yawd = x_(5);

            //avoid division by zero
            if (fabs(yawd) > 0.001) {
                x_p = x + v/yawd * ( sin (yaw + yawd * delta_T) - sin(yaw));
                y_p = y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_T) );
                yawd_p = yawd;
            } else {
                x_p = x + v * delta_T * cos(yaw);
                y_p = y + v * delta_T * sin(yaw);
                yawd_p = 0.001;
            }

            z_p = z + v * delta_T;
            v_p = v;
            yaw_p = yaw + yawd * delta_T;

            // Calculate the Jacobian of the Transition Matrix F for CTRV model
            F_.setIdentity();
            F_(0, 3) = (1.0 / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
            F_(0, 4) = (v_p / yawd_p) * (cos(yawd_p * delta_T + yaw_p) - cos(yaw_p));
            F_(0, 5) = (delta_T * v_p / yawd_p)*cos(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
            F_(1, 3) = (1.0 / yawd_p) * (cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));
            F_(1, 4) = (v_p / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
            F_(1, 5) = (delta_T * v_p / yawd_p) * sin(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));
            
            P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix
            break;
        case KFConstantTurnRate:
            // Use the Constant Turn Rate and Velocity Motion Model to predict new state
            // extract values for better readability
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v = x_(3);
            yaw = x_(4);
            yawd = x_(5);

            //avoid division by zero
            if (fabs(yawd) > 0.001) {
                x_p = x + v/yawd * ( sin (yaw + yawd * delta_T) - sin(yaw));
                y_p = y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_T) );
                yawd_p = yawd;
            } else {
                x_p = x + v * delta_T * cos(yaw);
                y_p = y + v * delta_T * sin(yaw);
                yawd_p = 0.001;
            }

            z_p = z + v * delta_T;
            v_p = v;
            yaw_p = yaw + yawd * delta_T;

            // Calculate the Jacobian of the Transition Matrix F for CTRV model
            F_.setIdentity();
            F_(0, 3) = (1.0 / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
            F_(0, 4) = (v_p / yawd_p) * (cos(yawd_p * delta_T + yaw_p) - cos(yaw_p));
            F_(0, 5) = (delta_T * v_p / yawd_p)*cos(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
            F_(1, 3) = (1.0 / yawd_p) * (cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));
            F_(1, 4) = (v_p / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
            F_(1, 5) = (delta_T * v_p / yawd_p) * sin(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));
            F_(4, 5) = delta_T; // CORRECTION
    
            // CORRECTION: had to update the current state with the predicitons
            x_ << x_p, y_p, z_p, v_p, yaw_p, yawd_p;
            
            P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix
            break;
        case KFConstantTurnRateWithZVelocity:
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v = x_(3);
            yaw = x_(4);
            yawd = x_(5);
            v_z = x_(6);

            //non linear motion model
            //avoid division by zero
            if (fabs(yawd) > 0.001) {
                x_p = x + v/yawd * ( sin (yaw + yawd * delta_T) - sin(yaw));
                y_p = y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_T) );
                yawd_p = yawd;
            } else {
                x_p = x + v * delta_T * cos(yaw);
                y_p = y + v * delta_T * sin(yaw);
                yawd_p = 0.001;
            }
            z_p = z + v_z * delta_T;
            v_z_p = v_z;
            v_p = v;
            yaw_p = yaw + yawd * delta_T;

            // Calculate the Jacobian of the non-linear motion model
            F_.setIdentity();
            F_(0, 3) = (1.0 / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));//ok
            F_(0, 4) = (v_p / yawd_p) * (cos(yawd_p * delta_T + yaw_p) - cos(yaw_p));//ok
            F_(0, 5) = (delta_T * v_p / yawd_p)*cos(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));//ok
            F_(1, 3) = (1.0 / yawd_p) * (cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));//ok
            F_(1, 4) = (v_p / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));//ok
            F_(1, 5) = (delta_T * v_p / yawd_p) * sin(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));//ok
            F_(2, 6) = delta_T;//ok
            F_(4, 5) = delta_T;//ok

            x_ << x_p, y_p, z_p, v_p, yaw_p, yawd_p, v_z_p; //update state

            P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix
            break;
        default:
            ROS_ERROR_STREAM("Unknown KF motion model type");
    }
}

void UAVTrackKF::update(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    ZoneScoped;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    check_with_KDTree(cloud, cloud_filtered);

    if (cloud_filtered->size() == 0) {
        ROS_WARN_STREAM("No points in the preset radius of previous position");
        return;
    }

    Eigen::VectorXd z = Eigen::VectorXd(3);

    getCloudCentroid(cloud_filtered, z);

    Eigen::VectorXd z_pred = H_ * x_; // compute predicted observation. Project estimated state THROUGH observation space

    Eigen::VectorXd y = z - z_pred; // residual

    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;

    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // Estimate new state
    x_ = x_ + K * y;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(),x_.size());
    P_ = (I - K * H_) * P_;
}

void UAVTrackKF::getCloudCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, Eigen::VectorXd& center) {
    ZoneScoped;
    if (cloud_in->size() > 0) {
        int n = cloud_in->size();
        double x = 0.0, y = 0.0, z = 0.0;
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud_in->begin(); it < cloud_in->end(); ++it) {
            x += it->x;
            y += it->y;
            z += it->z; 
        }
        x /= n;
        y /= n;
        z /= n;
        center << x, y, z;
    }
}

bool UAVTrackKF::is_first_measurement() {
    ZoneScoped;
    return first_measurement;
}

void UAVTrackKF::set_first_measurement(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    ZoneScoped;
    // get the centroid of the point cloud
    Eigen::VectorXd position = Eigen::VectorXd(3);
    getCloudCentroid(cloud, position);
    switch (motion_model_type) {
        case KFConstVelocityDogru:
        case KFConstantTurnRateCatalano:
        case KFConstantTurnRate:
            x_ << position(0), position(1), position(2), 0, 0, 0;
            break;
        case KFConstantTurnRateWithZVelocity:
            x_ << position(0), position(1), position(2), 0, 0, 0, 0;
            break;
        default:
            ROS_ERROR_STREAM("Unknown KF motion model type");
    }
    first_measurement = false;
}

void UAVTrackKF::check_with_KDTree(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
    ZoneScoped;
    // check if there are points in the preset radius of the previous position
    // Initialize KDTree for searching object in point cloud
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kd_tree->setInputCloud(cloud_in);

    // Define KDTree search parameters
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointXYZ position;

    position.x = x_(0);
    position.y = x_(1);
    position.z = x_(2);
    
    double search_r = this->search_radius;

    if (motion_model_type == KFConstVelocityDogru){
        //update search radius based on sigmas
        search_r = 3 * std::sqrt(std::max(P_(0, 0), std::max(P_(1, 1), P_(2, 2))));
        if (this->dogru_use_min_search_radius) {
            search_r = std::max(search_r, this->search_radius);
        }
    }

    // If neighbors within radius are found
    if(kd_tree->radiusSearch(position, search_r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            // Retrieve point belonging to object from point cloud based on index
            pcl::PointXYZ pt = (*cloud_in)[pointIdxRadiusSearch[i]];

            // Add point to object point cloud
            cloud_out->points.push_back(pt); 
        }
    }
}

visualization_msgs::Marker UAVTrackKF::getEstimate() {
    ZoneScoped;
    visualization_msgs::Marker uav_pose_marker;

    uav_pose_marker.header.frame_id = frame;
    uav_pose_marker.type = visualization_msgs::Marker::ARROW;
    uav_pose_marker.action = visualization_msgs::Marker::ADD;

    uav_pose_marker.points.resize(2);
    double x, y, z, v, yaw, yaw_rate, v_x, v_y, v_z;

    switch (motion_model_type) {
        case KFConstVelocityDogru:
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v_x = x_(3);
            v_y = x_(4);
            v_z = x_(5);
            break;
        case KFConstantTurnRateCatalano:
        case KFConstantTurnRate:
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v = x_(3);
            yaw = x_(4);
            yaw_rate = x_(5);
            v_x = v * cos(yaw);
            v_y = v * sin(yaw);
            v_z = 0.0;
            break;
        case KFConstantTurnRateWithZVelocity:
            x = x_(0);
            y = x_(1);
            z = x_(2);
            v = x_(3);
            yaw = x_(4);
            yaw_rate = x_(5);
            v_z = x_(6);
            v_x = v * cos(yaw);
            v_y = v * sin(yaw);
            break;
        default:
            ROS_ERROR_STREAM("Unknown KF motion model type");
    }

    uav_pose_marker.points[0].x = x;
    uav_pose_marker.points[0].y = y;
    uav_pose_marker.points[0].z = z;
    uav_pose_marker.points[1].x = x + v_x;
    uav_pose_marker.points[1].y = y + v_y;
    uav_pose_marker.points[1].z = z + v_z;

    uav_pose_marker.scale.x = 0.1;
    uav_pose_marker.scale.y = 0.1;
    uav_pose_marker.scale.z = 0.1;
    uav_pose_marker.color.r = 1.0;
    uav_pose_marker.color.a = 0.0;

    // Set time
    uav_pose_marker.header.stamp = ros::Time::now();

    return uav_pose_marker;
}

bool UAVTrackKF::get_is_tracked() {
    ZoneScoped;
    if (first_measurement) {
        return false;
    }
    if (P_(0, 0) < search_radius && P_(1, 1) < search_radius && P_(2, 2) < search_radius) {
        return true;
    } else {
        return false;
    }
}
