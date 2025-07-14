#include <ros/ros.h>

// ROS messages and services
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <jsk_rviz_plugins/OverlayText.h>

// TF
#include <tf/transform_listener.h>

// PointCloud
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

// OctoMap
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

// Tracy
#include "tracy/Tracy.hpp"

// Custom headers
#include "mav_track/BuildOctoMapBackground.h" // This is auto-generated!
#include "mav_track/mav_track_particlefilter.hpp"
#include "mav_track/mav_track_KF.hpp"

enum TrackingType {
    ParticleFilter = 0,
    EKF = 1
};

enum BackgroundModelType {
    NoBackground = 0,
    Octomap = 1,
    KriszSegmentation = 2
};

class MAVTracker {
  private:
    // ROS NodeHandle
    ros::NodeHandle nh;

    // Params
    double frequency;
    double delta_time;
    std::string frame;
    std::string topic_LidarCloud;
    TrackingType tracking_type;
    int tracking_type_int;
    BackgroundModelType background_model_type;
    int background_model_type_int;
    ros::Rate* rate;
    std::string service_BuildOctoMap_name;
    double octomap_resolution;
    int octomap_build_num_frames;
    bool filter_distance_min;
    double filter_distance_min_value;
    bool filter_distance_max;
    double filter_distance_max_value;
    bool filter_BBX;
    std::vector<double> filter_BBX_min;
    std::vector<double> filter_BBX_max;
    int octomap_search_depth;
    bool pcl_filter_outlier_radius;
    double pcl_filter_outlier_radius_value_RadiusSearch;
    int pcl_filter_outlier_radius_value_MinNeighborsInRadius;
    bool pcl_filter_outlier_statistical;
    int pcl_filter_outlier_statistical_value_MeanK;
    double pcl_filter_outlier_statistical_value_StddevMulThresh;
    std::string topic_FilteredCloud;
    int particle_filter_Nparticles;
    float particle_filter_initial_sampling_zone_radius;
    float particle_filter_initial_sampling_zone_height;
    float particle_filter_initial_sampling_zone_x;
    float particle_filter_initial_sampling_zone_y;
    float particle_filter_initial_sampling_zone_z;
    PFMotionModelType particle_filter_motion_model_type;
    int particle_filter_motion_model_type_int;
    float particle_filter_predict_std_pos;
    float particle_filter_predict_std_vel;
    float particle_filter_predict_std_acc;
    float particle_filter_sensing_std;
    bool particle_filter_velocity_when_not_tracked;
    bool particle_filter_acceleration_when_not_tracked;
    std::string topic_MAVDistance;
    std::string topic_MAVMarker;
    std::string topic_ParticleMarker;
    std::string topic_FilterBBX;
    std::string service_ParticleReset;
    float KF_sigma_lidar;
    float KF_sigma_acc;
    float KF_search_radius;
    KFMotionModelType KF_motion_model_type;
    int KF_motion_model_type_int;
    std::string service_KFReset;
    bool KF_dogru_use_min_search_radius;

    // Variables
    // Octree background model, if BackgroundModelType is Octomap
    octomap::OcTree* bg_model;
    // The point cloud after background subtraction and filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mav;
    // keeps track if new measurement arrived
    bool new_measurement;
    // Particle filter for MAV tracking (even if not used for tracking)
    MAVTrackParticleFilter pfMAV;
    // Kalman filter for MAV tracking
    MAVTrackKF kfMAV;
    // estimated MAV pose with velocity
    visualization_msgs::Marker mav_velocity_marker;

    // Services, subscribers, and publishers
    ros::ServiceClient srv_BuildOctoMap;
    tf::TransformListener transform_listener;
    ros::Subscriber sub_LidarCloud;
    ros::Publisher pub_FilteredCloud;
    ros::Publisher pub_MAVDistance;
    ros::Publisher pub_MAVMarker;
    ros::Publisher pub_ParticleMarker;
    ros::Publisher pub_FilterBBX;
    ros::ServiceServer srv_ParticleReset;
    ros::ServiceServer srv_KFReset;
    
  public:
    MAVTracker();
    ~MAVTracker();

    // Callbacks
    void callback_LidarCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool callback_ParticleReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
    bool callback_KFReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

    // Publish functions
    void publish_FilteredCloud();
    void publish_ParticleMarker();
    void publish_BoundingBox();
    void publish_MAV(bool is_tracked = true);

    // Service calls
    bool call_service_BuildOctoMap(int num_frames);

    //other functions
    void loop();
    void execute();
};

MAVTracker::MAVTracker() : nh("~") {
    // Initialize parameters
    frequency = 10.0;
    frame = "world";
    topic_LidarCloud = "/livox/lidar";
    tracking_type_int = ParticleFilter;
    background_model_type_int = Octomap;
    filter_distance_min = true;
    filter_distance_min_value = 1.0;
    filter_distance_max = false;
    filter_distance_max_value = 200.0;
    filter_BBX = true;
    filter_BBX_min.assign(3, -10.0);
    filter_BBX_max.assign(3, 10.0);
    pcl_filter_outlier_radius = true;
    pcl_filter_outlier_radius_value_RadiusSearch = 0.1;
    pcl_filter_outlier_radius_value_MinNeighborsInRadius = 8;
    pcl_filter_outlier_statistical = true;
    pcl_filter_outlier_statistical_value_MeanK = 4;
    pcl_filter_outlier_statistical_value_StddevMulThresh = 1.0;
    topic_FilteredCloud = "/mav_track/filtered_cloud";
    topic_MAVDistance = "/mav_track/mav_distance";
    topic_MAVMarker = "/mav_track/mav_marker";    
    topic_FilterBBX = "/mav_track/filter_BBX";
    
    nh.getParam("Frequency", frequency);
    nh.getParam("Frame", frame);
    nh.getParam("Topic_Sub_LidarCloud", topic_LidarCloud);
    nh.getParam("Filter_Distance_Min", filter_distance_min);
    nh.getParam("Filter_Distance_Min_Value", filter_distance_min_value);
    nh.getParam("Filter_Distance_Max", filter_distance_max);
    nh.getParam("Filter_Distance_Max_Value", filter_distance_max_value);
    nh.getParam("Filter_BBX", filter_BBX);
    nh.getParam("Filter_BBX_min", filter_BBX_min);
    nh.getParam("Filter_BBX_max", filter_BBX_max);
    nh.getParam("Tracking_Type", tracking_type_int);
    tracking_type = static_cast<TrackingType>(tracking_type_int);
    nh.getParam("PCL_Filter_Outlier_Radius", pcl_filter_outlier_radius);
    nh.getParam("PCL_Filter_Outlier_Radius_Value_RadiusSearch", pcl_filter_outlier_radius_value_RadiusSearch);
    nh.getParam("PCL_Filter_Outlier_Radius_Value_MinNeighborsInRadius", pcl_filter_outlier_radius_value_MinNeighborsInRadius);
    nh.getParam("PCL_Filter_Outlier_Statistical", pcl_filter_outlier_statistical);
    nh.getParam("PCL_Filter_Outlier_Statistical_Value_MeanK", pcl_filter_outlier_statistical_value_MeanK);
    nh.getParam("PCL_Filter_Outlier_Statistical_Value_StddevMulThresh", pcl_filter_outlier_statistical_value_StddevMulThresh);
    nh.getParam("Background_Model_Type", background_model_type_int);
    background_model_type = static_cast<BackgroundModelType>(background_model_type_int);
    nh.getParam("Topic_Pub_FilteredCloud", topic_FilteredCloud);
    nh.getParam("Topic_Pub_MAVDistance", topic_MAVDistance);
    nh.getParam("Topic_Pub_MAVMarker", topic_MAVMarker);
    nh.getParam("Topic_Pub_FilterBBX", topic_FilterBBX);
   
    ROS_INFO_STREAM("MAVTracker Params:");
    ROS_INFO_STREAM("  Frequency: " << frequency);
    ROS_INFO_STREAM("  Frame: " << frame);
    ROS_INFO_STREAM("  Topic_Sub_LidarCloud: " << topic_LidarCloud);
    ROS_INFO_STREAM("  Filter_Distance_Min: " << filter_distance_min);
    ROS_INFO_STREAM("  Filter_Distance_Min_Value: " << filter_distance_min_value);
    ROS_INFO_STREAM("  Filter_Distance_Max: " << filter_distance_max);
    ROS_INFO_STREAM("  Filter_Distance_Max_Value: " << filter_distance_max_value);
    ROS_INFO_STREAM("    Filter_BBX: " << filter_BBX);
    ROS_INFO_STREAM("    Filter_BBX_min: [" 
        << filter_BBX_min[0] << ", "
        << filter_BBX_min[1] << ", "
        << filter_BBX_min[2] << "]");
    ROS_INFO_STREAM("    Filter_BBX_max: [" 
        << filter_BBX_max[0] << ", "
        << filter_BBX_max[1] << ", "
        << filter_BBX_max[2] << "]");
    ROS_INFO_STREAM("  TrackingType: " << tracking_type);
    ROS_INFO_STREAM("  PCL_Filter_Outlier_Radius: " << pcl_filter_outlier_radius);
    ROS_INFO_STREAM("  PCL_Filter_Outlier_Radius_Value_RadiusSearch: " << pcl_filter_outlier_radius_value_RadiusSearch);
    ROS_INFO_STREAM("  PCL_Filter_Outlier_Radius_Value_MinNeighborsInRadius: " << pcl_filter_outlier_radius_value_MinNeighborsInRadius);
    ROS_INFO_STREAM("  PCL_Filter_Outlier_Statistical: " << pcl_filter_outlier_statistical);
    ROS_INFO_STREAM("  PCL_Filter_Outlier_Statistical_Value_MeanK: " << pcl_filter_outlier_statistical_value_MeanK);
    ROS_INFO_STREAM("  PCL_Filter_Outlier_Statistical_Value_StddevMulThresh: " << pcl_filter_outlier_statistical_value_StddevMulThresh);
    ROS_INFO_STREAM("  BackgroundModelType: " << background_model_type);
    ROS_INFO_STREAM("  Topic_Pub_FilteredCloud: " << topic_FilteredCloud);
    ROS_INFO_STREAM("  Topic_Pub_MAVDistance: " << topic_MAVDistance);
    ROS_INFO_STREAM("  Topic_Pub_MAVMarker: " << topic_MAVMarker);
    ROS_INFO_STREAM("  Topic_Pub_FilterBBX: " << topic_FilterBBX);

    int queue_sub = 1;
    int queue_pub = 1;
    
    // set rate
    rate = new ros::Rate(frequency);
    delta_time = 1.0 / frequency;

    // Background model initialization
    if (background_model_type == Octomap) {
        ROS_INFO_STREAM("Background model type: Octomap");
        
        // Initialize parameters
        octomap_resolution = 0.1;
        octomap_search_depth = 0;
        octomap_build_num_frames = 100;
        service_BuildOctoMap_name = "/mav_track/build_octomap_background";
        
        nh.getParam("Octomap_Resolution", octomap_resolution);
        nh.getParam("Octomap_SearchDepth", octomap_search_depth);
        nh.getParam("Octomap_Build_Num_Frames", octomap_build_num_frames);
        nh.getParam("Service_BuildOctoMap_name", service_BuildOctoMap_name); 
        
        ROS_INFO_STREAM("  Octomap_Resolution: " << octomap_resolution);
        ROS_INFO_STREAM("  Octomap_SearchDepth: " << octomap_search_depth);
        ROS_INFO_STREAM("  Octomap_Build_Num_Frames: " << octomap_build_num_frames);
        ROS_INFO_STREAM("  Service_BuildOctoMap: " << service_BuildOctoMap_name);

        bg_model = new octomap::OcTree(octomap_resolution);

        // Services, subscribers, and publishers

        srv_BuildOctoMap = nh.serviceClient<mav_track::BuildOctoMapBackground>(service_BuildOctoMap_name);
    }

    // Tracking type ParticleFilter
    if (tracking_type == ParticleFilter) {
        ROS_INFO_STREAM("Tracking type: ParticleFilter");
        // Initialize parameters
        particle_filter_Nparticles = 1000;
        particle_filter_initial_sampling_zone_radius = 5.0;
        particle_filter_initial_sampling_zone_height = 2.0;
        particle_filter_initial_sampling_zone_x = 0.0;
        particle_filter_initial_sampling_zone_y = 0.0;
        particle_filter_initial_sampling_zone_z = 0.0;
        particle_filter_motion_model_type_int = 0;
        particle_filter_predict_std_pos = 0.1;
        particle_filter_predict_std_vel = 0.1;
        particle_filter_predict_std_acc = 0.1;
        particle_filter_sensing_std = 0.5;
        particle_filter_velocity_when_not_tracked = false;
        particle_filter_acceleration_when_not_tracked = false;
        topic_ParticleMarker = "/mav_track/particles";
        service_ParticleReset = "/mav_track/reset_particles";

        nh.getParam("ParticleFilter_Nparticles", particle_filter_Nparticles);
        nh.getParam("ParticleFilter_Initial_Sampling_Zone_Radius", particle_filter_initial_sampling_zone_radius);
        nh.getParam("ParticleFilter_Initial_Sampling_Zone_Height", particle_filter_initial_sampling_zone_height);
        nh.getParam("ParticleFilter_Initial_Sampling_Zone_X", particle_filter_initial_sampling_zone_x);
        nh.getParam("ParticleFilter_Initial_Sampling_Zone_Y", particle_filter_initial_sampling_zone_y);
        nh.getParam("ParticleFilter_Initial_Sampling_Zone_Z", particle_filter_initial_sampling_zone_z);
        nh.getParam("ParticleFilter_Motion_Model_Type", particle_filter_motion_model_type_int);
        particle_filter_motion_model_type = static_cast<PFMotionModelType>(particle_filter_motion_model_type_int);
        nh.getParam("ParticleFilter_Predict_Std_Pos", particle_filter_predict_std_pos);
        nh.getParam("ParticleFilter_Predict_Std_Vel", particle_filter_predict_std_vel);
        nh.getParam("ParticleFilter_Predict_Std_Acc", particle_filter_predict_std_acc);
        nh.getParam("ParticleFilter_Sensing_Std", particle_filter_sensing_std);
        nh.getParam("ParticleFilter_Velocity_When_Not_Tracked", particle_filter_velocity_when_not_tracked);
        nh.getParam("ParticleFilter_Acceleration_When_Not_Tracked", particle_filter_acceleration_when_not_tracked);
        nh.getParam("Topic_Pub_ParticleMarker", topic_ParticleMarker);
        nh.getParam("Service_ParticleReset", service_ParticleReset);

        ROS_INFO_STREAM("  ParticleFilter_Nparticles: " << particle_filter_Nparticles);
        ROS_INFO_STREAM("  ParticleFilter_Initial_Sampling_Zone_Radius: " << particle_filter_initial_sampling_zone_radius);
        ROS_INFO_STREAM("  ParticleFilter_Initial_Sampling_Zone_Height: " << particle_filter_initial_sampling_zone_height);
        ROS_INFO_STREAM("  ParticleFilter_Initial_Sampling_Zone_X: " << particle_filter_initial_sampling_zone_x);
        ROS_INFO_STREAM("  ParticleFilter_Initial_Sampling_Zone_Y: " << particle_filter_initial_sampling_zone_y);
        ROS_INFO_STREAM("  ParticleFilter_Initial_Sampling_Zone_Z: " << particle_filter_initial_sampling_zone_z);
        ROS_INFO_STREAM("  ParticleFilter_Motion_Model_Type: " << particle_filter_motion_model_type);
        ROS_INFO_STREAM("  ParticleFilter_Predict_Std_Pos: " << particle_filter_predict_std_pos);
        ROS_INFO_STREAM("  ParticleFilter_Predict_Std_Vel: " << particle_filter_predict_std_vel);
        ROS_INFO_STREAM("  ParticleFilter_Predict_Std_Acc: " << particle_filter_predict_std_acc);
        ROS_INFO_STREAM("  ParticleFilter_Sensing_Std: " << particle_filter_sensing_std);
        if (!particle_filter_velocity_when_not_tracked && particle_filter_acceleration_when_not_tracked){
            ROS_WARN_STREAM("  ParticleFilter_Acceleration_When_Not_Tracked is set to true, but ParticleFilter_Velocity_When_Not_Tracked is false."
                " Ignoring ParticleFilter_Acceleration_When_Not_Tracked setting.");
            particle_filter_acceleration_when_not_tracked = false;
        }
        ROS_INFO_STREAM("  ParticleFilter_Velocity_When_Not_Tracked: " << particle_filter_velocity_when_not_tracked);
        ROS_INFO_STREAM("  ParticleFilter_Acceleration_When_Not_Tracked: " << particle_filter_acceleration_when_not_tracked);
        ROS_INFO_STREAM("  Topic_Pub_ParticleMarker: " << topic_ParticleMarker);
        ROS_INFO_STREAM("  Service_ParticleReset: " << service_ParticleReset);

        // Initialize particle filter
        pfMAV.set_Nparticles(particle_filter_Nparticles);
        pfMAV.set_initial_sampling_zone(
            particle_filter_initial_sampling_zone_radius,
            particle_filter_initial_sampling_zone_height,
            particle_filter_initial_sampling_zone_x,
            particle_filter_initial_sampling_zone_y,
            particle_filter_initial_sampling_zone_z);
        pfMAV.set_default_predict_std(
            particle_filter_predict_std_pos,
            particle_filter_predict_std_vel,
            particle_filter_predict_std_acc);
        pfMAV.set_default_sensing_std(particle_filter_sensing_std);
        pfMAV.set_motion_model_type(
            particle_filter_motion_model_type,
            particle_filter_velocity_when_not_tracked,
            particle_filter_acceleration_when_not_tracked);
        pfMAV.set_frame(frame);
        pfMAV.set_frequency(frequency);
        pfMAV.particles_initialize();

        pub_ParticleMarker = nh.advertise<visualization_msgs::Marker>(topic_ParticleMarker, queue_pub);
        srv_ParticleReset = nh.advertiseService(service_ParticleReset, &MAVTracker::callback_ParticleReset, this);
    } else if (tracking_type == EKF) {
        ROS_INFO_STREAM("Tracking type: EKF");
        // Initialize parameters
        KF_sigma_lidar = 0.1;
        KF_sigma_acc = 0.1;
        KF_search_radius = 0.2;
        service_KFReset = "/mav_track/reset_kf";
        KF_motion_model_type_int = 0;
        KF_dogru_use_min_search_radius = false;

        nh.getParam("KF_Sigma_Lidar", KF_sigma_lidar);
        nh.getParam("KF_Sigma_Acc", KF_sigma_acc);
        nh.getParam("KF_Search_Radius", KF_search_radius);
        nh.getParam("Service_KFReset", service_KFReset);
        nh.getParam("KF_Motion_Model_Type", KF_motion_model_type_int);
        KF_motion_model_type = static_cast<KFMotionModelType>(KF_motion_model_type_int);
        

        ROS_INFO_STREAM("  KF_Sigma_Lidar: " << KF_sigma_lidar);
        ROS_INFO_STREAM("  KF_Sigma_Acc: " << KF_sigma_acc);
        ROS_INFO_STREAM("  KF_Search_Radius: " << KF_search_radius);
        ROS_INFO_STREAM("  Service_KFReset: " << service_KFReset);
        ROS_INFO_STREAM("  KF_Motion_Model_Type: " << KF_motion_model_type);        

        if (KF_motion_model_type == KFConstVelocityDogru) {
            ROS_INFO_STREAM("  KF_Motion_Model_Type: KFConstVelocity");
            nh.getParam("KF_Dogru_Use_Min_Search_Radius", KF_dogru_use_min_search_radius);
            ROS_INFO_STREAM("  KF_Dogru_Use_Min_Search_Radius: " << KF_dogru_use_min_search_radius);

            Eigen::VectorXd x_in = Eigen::VectorXd::Zero(6);
            x_in << 0, 0, 0, 0, 0, 0;
            Eigen::MatrixXd P_in = Eigen::MatrixXd::Identity(6, 6);
            
            Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(6,6);

            Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3,6);
            H_in(0, 0) = 1;
            H_in(1, 1) = 1;
            H_in(2, 2) = 1;

            Eigen::MatrixXd R_in = Eigen::MatrixXd::Identity(3,3) * KF_sigma_lidar * KF_sigma_lidar;

            Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(6,6) * KF_sigma_acc * KF_sigma_acc;

            kfMAV.init(x_in, P_in, F_in, H_in, R_in, Q_in, KF_search_radius, KF_motion_model_type ,frame, KF_dogru_use_min_search_radius);

        } else if (KF_motion_model_type == KFConstantTurnRate || KF_motion_model_type == KFConstantTurnRateCatalano) {
            ROS_INFO_STREAM("  KF_Motion_Model_Type: KFConstantTurnRate");
            Eigen::VectorXd x_in = Eigen::VectorXd::Zero(6);
            x_in << 0, 0, 0, 0, 0, 0;
            Eigen::MatrixXd P_in;
            Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(6, 6);
            // covariance matrix
            if (KF_motion_model_type == KFConstantTurnRateCatalano) {
                // hardcoded here from their paper
                P_in = Eigen::MatrixXd::Zero(6, 6);
                P_in(0, 0) = 1;
                P_in(1, 1) = 1;
                P_in(2, 2) = 1;
                P_in(3, 3) = 50 * 50;
                P_in(4, 4) = 50 * 50;
                P_in(5, 5) = 5 * 5;
                F_in(0, 3) = 1;
                F_in(1, 4) = 1;
                F_in(2, 5) = 1;
            } else {
                P_in = Eigen::MatrixXd::Identity(6, 6);
            }            

            Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3,6); // the matrix has this shape because the sensor is a LiDAR
            H_in(0, 0) = 1;
            H_in(1, 1) = 1;
            H_in(2, 2) = 1;

            Eigen::MatrixXd R_in = Eigen::MatrixXd::Identity(3,3) * KF_sigma_lidar * KF_sigma_lidar;

            Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(6,6) * KF_sigma_acc * KF_sigma_acc;

            kfMAV.init(x_in, P_in, F_in, H_in, R_in, Q_in, KF_search_radius, KF_motion_model_type ,frame);
        } else if (KF_motion_model_type == KFConstantTurnRateWithZVelocity) {
            ROS_INFO_STREAM("  KF_Motion_Model_Type: KFConstantTurnRateAndVelocity");
            Eigen::VectorXd x_in = Eigen::VectorXd::Zero(7);
            x_in << 0, 0, 0, 0, 0, 0, 0;

            // covariance matrix
            Eigen::MatrixXd P_in = Eigen::MatrixXd::Identity(7, 7);

            // Jakobian matrix will be stored here
            Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(7,7);

            // state mapping to sensor space
            Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3,7);
            H_in(0, 0) = 1;
            H_in(1, 1) = 1;
            H_in(2, 2) = 1;

            // Measurement noise covariance
            Eigen::MatrixXd R_in = Eigen::MatrixXd::Identity(3,3) * KF_sigma_lidar * KF_sigma_lidar;

            // Process noise covariance
            Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(7,7) * KF_sigma_acc * KF_sigma_acc;

            kfMAV.init(x_in, P_in, F_in, H_in, R_in, Q_in, KF_search_radius, KF_motion_model_type ,frame);
        } else {
            ROS_ERROR_STREAM("  KF_Motion_Model_Type: Unknown motion model type");
        }
        srv_KFReset = nh.advertiseService(service_KFReset, &MAVTracker::callback_KFReset, this);
    }

    // Initialize the background model
    if (background_model_type == Octomap) {
        // Wait until the BuildOctoMap service is available
        ROS_INFO_STREAM("Waiting for service " << service_BuildOctoMap_name << " to be advertised...");
        srv_BuildOctoMap.waitForExistence();
        ROS_INFO_STREAM("Service " << service_BuildOctoMap_name << " is now available.");

        if(!call_service_BuildOctoMap(octomap_build_num_frames)) {
            ROS_ERROR("Failed to build octomap background model.");
            return;
        }
    }

    // Initialize variables
    cloud_mav.reset(new pcl::PointCloud<pcl::PointXYZ>());
    new_measurement = false;

    if (filter_BBX){
        ROS_INFO_STREAM("Filter_BBX is enabled. This overrides the filter_distance_min and filter_distance_max settings.");
        filter_distance_min = false;
        filter_distance_max = false;
    }

    // Services, subscribers, and publishers
    sub_LidarCloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_LidarCloud, queue_sub, &MAVTracker::callback_LidarCloud, this);
    pub_FilteredCloud = nh.advertise<sensor_msgs::PointCloud2>(topic_FilteredCloud, queue_pub);
    pub_MAVDistance = nh.advertise<jsk_rviz_plugins::OverlayText>(topic_MAVDistance, queue_pub);
    pub_MAVMarker = nh.advertise<visualization_msgs::Marker>(topic_MAVMarker, queue_pub);
    pub_FilterBBX = nh.advertise<visualization_msgs::Marker>(topic_FilterBBX, queue_pub);

    // Print ROS info message
    ROS_INFO_STREAM("mav_tracker initialization finished.\n-----------------------------------------");
}

MAVTracker::~MAVTracker() {
    // see https://github.com/OctoMap/octomap_mapping
    if (bg_model) {
        bg_model->clear();
        delete bg_model;
        bg_model = NULL;
    }
}

bool MAVTracker::call_service_BuildOctoMap(int num_frames) {
    ZoneScoped;
    mav_track::BuildOctoMapBackground srv;
    srv.request.num_frames = num_frames;

    if (srv_BuildOctoMap.call(srv)) {
        ROS_INFO("BuildOctoMap service call succeeded.");
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(srv.response.map);

        if (abstract_tree) {
            delete bg_model;
            bg_model = dynamic_cast<octomap::OcTree*>(abstract_tree);
            if (bg_model) {
                ROS_INFO("Received octomap with frame_id: %s", srv.response.map.header.frame_id.c_str());
                return true;
            } else {
                ROS_ERROR("Error casting to OcTree*");
                delete abstract_tree;
                return false;
            }
        } else {
            ROS_ERROR("Failed to convert octomap message");
            return false;
        }
    }
    return false;
}

// Filter the point cloud and publish the potential MAV cloud
void MAVTracker::callback_LidarCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ZoneScoped;
    // first check if transform of cloud is available
    {
        ZoneScopedN("wait_transform");
        if (!transform_listener.waitForTransform(frame, msg->header.frame_id, msg->header.stamp, ros::Duration(delta_time/2.0))) {
            ROS_WARN_STREAM("Was not able to lookup transform from [" << msg->header.frame_id << "] to [" << frame << "].");
            return;
        }
    }
    // transform pointcloud into global frame
    sensor_msgs::PointCloud2 pcl_transformed;
    pcl_ros::transformPointCloud(frame, *msg, pcl_transformed, transform_listener);
    {
        ZoneScopedN("clear");
        // subtract background from pointcloud
        cloud_mav->clear();
        cloud_mav->header.frame_id = frame;
    }

    // if filter_distance is used, calculate min and max distance (only if pcl_filter_BBX is not used)
    double distance_min, distance_max;
    if (filter_distance_min || filter_distance_max) {
        distance_min = filter_distance_min_value;
        distance_max = filter_distance_max_value;
        distance_min *= distance_min; // squared in advance for efficiency
        distance_max *= distance_max; // squared in advance for efficiency
    }

    {
        ZoneScopedN("iterate");
        std::string zone_text = "pcl_transformed: " + std::to_string(pcl_transformed.data.size());
        ZoneText(zone_text.c_str(), strlen(zone_text.c_str()));
        for (sensor_msgs::PointCloud2ConstIterator<float> it(pcl_transformed, "x"); it != it.end(); ++it) {
            // filter: remove ground plane or ceiling
            if (filter_BBX) {
                if (it[0] < filter_BBX_min[0] || it[0] > filter_BBX_max[0])
                    continue;
                if (it[1] < filter_BBX_min[1] || it[1] > filter_BBX_max[1])
                    continue;
                if (it[2] < filter_BBX_min[2] || it[2] > filter_BBX_max[2])
                    continue;
            }

            geometry_msgs::PointStamped origin_lidar;
            origin_lidar.header = msg->header;
            origin_lidar.point.x = origin_lidar.point.y = origin_lidar.point.z = 0.0;
            transform_listener.transformPoint(frame, origin_lidar, origin_lidar);

            // filter: remove near or far points (only if pcl_filter_BBX is not used)
            if (filter_distance_min || filter_distance_max) {
                double distance_squared = (it[0] - origin_lidar.point.x)*(it[0] - origin_lidar.point.x)
                                        + (it[1] - origin_lidar.point.y)*(it[1] - origin_lidar.point.y)
                                        + (it[2] - origin_lidar.point.z)*(it[2] - origin_lidar.point.z);
                if (filter_distance_min && distance_squared < distance_min)
                    continue;
                if (filter_distance_max && distance_squared > distance_max)
                    continue;
            }

            if (background_model_type == Octomap) {
                // set octree search depth, depth=0 results in full tree depth (same as depth=getTreeDepth())
                unsigned int depth = bg_model->getTreeDepth() - octomap_search_depth;
                // check if point is occupied in octomap
                octomap::OcTreeNode* onode = bg_model->search(it[0], it[1], it[2], depth);
                bool add_point = true;
                //if its part of the background
                if (onode != NULL && bg_model->isNodeOccupied(onode))
                    add_point = false;
                if (add_point)
                    cloud_mav->push_back(pcl::PointXYZ(it[0], it[1], it[2]));
            } else {
                cloud_mav->push_back(pcl::PointXYZ(it[0], it[1], it[2]));
            }
        }
    }
    // filter: RadiusOutlierRemoval
    if (pcl_filter_outlier_radius && cloud_mav->size() > 0) {
        // remove outliers by radius
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud_mav);
        outrem.setRadiusSearch(pcl_filter_outlier_radius_value_RadiusSearch);
        outrem.setMinNeighborsInRadius(pcl_filter_outlier_radius_value_MinNeighborsInRadius);
        // apply filter
        outrem.filter(*cloud_mav);
    }

    // filter: StatisticalOutlierRemoval
    if (pcl_filter_outlier_statistical && cloud_mav->size() > 0) {
        // see https://pointclouds.org/documentation/group__filters.html
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter;
        sorfilter.setInputCloud(cloud_mav);
        sorfilter.setMeanK(pcl_filter_outlier_statistical_value_MeanK);
        sorfilter.setStddevMulThresh(pcl_filter_outlier_statistical_value_StddevMulThresh);
        sorfilter.filter(*cloud_mav);
    }

    // set variable if new measurement arrived
    if (cloud_mav->size() > 0){
        publish_FilteredCloud();
        new_measurement = true;
    }
}

// reset particles of filter
bool MAVTracker::callback_ParticleReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res) {
    ZoneScoped;
    // req is empty in trigger request
    res.message = "Triggered particle reset.";
    pfMAV.particles_initialize();
    res.success = true;
    return true;
}

// reset Kalman filter
bool MAVTracker::callback_KFReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res) {
    ZoneScoped;
    // req is empty in trigger request
    res.message = "Triggered Kalman Filter reset.";
    
    if (KF_motion_model_type == KFConstVelocityDogru) {
        Eigen::VectorXd x_in = Eigen::VectorXd::Zero(6);
        x_in << 0, 0, 0, 0, 0, 0;
        Eigen::MatrixXd P_in = Eigen::MatrixXd::Identity(6, 6);
        
        Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(6,6);

        Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3,6); // the matrix has this shape because the sensor is a LiDAR
        H_in(0, 0) = 1;
        H_in(1, 1) = 1;
        H_in(2, 2) = 1;

        Eigen::MatrixXd R_in = Eigen::MatrixXd::Identity(3,3) * KF_sigma_lidar * KF_sigma_lidar;

        Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(6,6) * KF_sigma_acc * KF_sigma_acc;

        kfMAV.init(x_in, P_in, F_in, H_in, R_in, Q_in, KF_search_radius, KF_motion_model_type ,frame, KF_dogru_use_min_search_radius);
    } else if (KF_motion_model_type == KFConstantTurnRate || KF_motion_model_type == KFConstantTurnRateCatalano) {
        Eigen::VectorXd x_in = Eigen::VectorXd::Zero(6);
        x_in << 0, 0, 0, 0, 0, 0;

        Eigen::MatrixXd P_in;
        // covariance matrix
        if (KF_motion_model_type == KFConstantTurnRateCatalano) {
            //hardcoded here
            P_in = Eigen::MatrixXd::Zero(6, 6);
            P_in(0, 0) = 1;
            P_in(1, 1) = 1;
            P_in(2, 2) = 1;
            P_in(3, 3) = 50 * 50; // fill velocity covariance error part using lidar parameters
            P_in(4, 4) = 50 * 50;
            P_in(5, 5) = 5 * 5;
        } else {
            P_in = Eigen::MatrixXd::Identity(6, 6);
        }

        Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(6,6);
        F_in(0, 3) = 1;
        F_in(1, 4) = 1;
        F_in(2, 5) = 1;

        Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3,6); // the matrix has this shape because the sensor is a LiDAR
        H_in(0, 0) = 1;
        H_in(1, 1) = 1;
        H_in(2, 2) = 1;

        Eigen::MatrixXd R_in = Eigen::MatrixXd::Identity(3,3) * KF_sigma_lidar * KF_sigma_lidar;

        Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(6,6) * KF_sigma_acc * KF_sigma_acc;

        kfMAV.init(x_in, P_in, F_in, H_in, R_in, Q_in, KF_search_radius, KF_motion_model_type ,frame);
    } else if (KF_motion_model_type == KFConstantTurnRateWithZVelocity) {
        Eigen::VectorXd x_in = Eigen::VectorXd::Zero(7);
        x_in << 0, 0, 0, 0, 0, 0, 0;

        // covariance matrix
        Eigen::MatrixXd P_in = Eigen::MatrixXd::Identity(7, 7);

        // Jakobian matrix will be stored here
        Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(7,7);

        // state mapping to sensor space
        Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3,7);
        H_in(0, 0) = 1;
        H_in(1, 1) = 1;
        H_in(2, 2) = 1;
        
        // Measurement noise covariance
        Eigen::MatrixXd R_in = Eigen::MatrixXd::Identity(3,3) * KF_sigma_lidar * KF_sigma_lidar;

        // Process noise covariance
        Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(7,7) * KF_sigma_acc * KF_sigma_acc;

        kfMAV.init(x_in, P_in, F_in, H_in, R_in, Q_in, KF_search_radius, KF_motion_model_type ,frame);
    } else {
        ROS_ERROR_STREAM("KF_Motion_Model_Type: Unknown motion model type");
    }

    res.success = true;
    return true;
}

void MAVTracker::publish_FilteredCloud() {
    ZoneScoped;
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*cloud_mav, message);
    message.header.stamp=ros::Time::now();
    pub_FilteredCloud.publish(message);
}

void MAVTracker::publish_MAV(bool is_tracked) {
    ZoneScoped;
    visualization_msgs::Marker mav_model_marker;
    mav_model_marker.header = mav_velocity_marker.header;
    mav_model_marker.ns = "mav_model";
    mav_model_marker.id = 0;
    mav_model_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mav_model_marker.action = visualization_msgs::Marker::ADD;
    mav_model_marker.pose.position = mav_velocity_marker.points[0];
    mav_model_marker.pose.orientation.w = 1.0;
    mav_model_marker.mesh_resource = "package://mav_track/meshes/firefly.dae";
    mav_model_marker.mesh_use_embedded_materials = true;
    mav_model_marker.scale.x = 1.0;
    mav_model_marker.scale.y = 1.0;
    mav_model_marker.scale.z = 1.0;
    
    // if not tracked, we publish the new model and velocity with alpha set to 0.0
    // so that the model and velocity vector is not visible in rviz
    std::string is_tracked_str = "";
    if (is_tracked) {
        // set alpha to 0.0 if not tracked 
        mav_velocity_marker.color.a = 1.0;
        mav_model_marker.color.a = 1.0;
        is_tracked_str = " (tracked)";
    } else {
        mav_velocity_marker.color.a = 0.0;
        mav_model_marker.color.a = 0.0;
        is_tracked_str = " (not tracked)";
    }

    pub_MAVMarker.publish(mav_model_marker);

    jsk_rviz_plugins::OverlayText mav_distance_msg = jsk_rviz_plugins::OverlayText();
    std::stringstream ss;
    ss << "MAV distance: " << 
        sqrt(pow(mav_velocity_marker.points[0].x,2)
        + pow(mav_velocity_marker.points[0].y,2)
        + pow(mav_velocity_marker.points[0].z,2)) << " m" << is_tracked_str;
    mav_distance_msg.text = ss.str();
    mav_distance_msg.top = 0;
    mav_distance_msg.left = 0;
    mav_distance_msg.fg_color.a = 1.0;
    mav_distance_msg.fg_color.b = 1.0;
    mav_distance_msg.fg_color.g = 1.0;
    mav_distance_msg.fg_color.r = 1.0;
    mav_distance_msg.bg_color.a = 0.0;
    mav_distance_msg.width = 600;
    mav_distance_msg.height = 100;
    mav_distance_msg.text_size = 20;

    pub_MAVDistance.publish(mav_distance_msg);

    if (tracking_type == EKF) {
        mav_velocity_marker.ns = "mav_velocity";
        mav_velocity_marker.id = 1;
        pub_MAVMarker.publish(mav_velocity_marker);
    } else if (tracking_type == ParticleFilter) {
        if (particle_filter_motion_model_type != OnlyPosition) {
            mav_velocity_marker.ns = "mav_velocity";
            mav_velocity_marker.id = 1;
            pub_MAVMarker.publish(mav_velocity_marker);
        }
    }
}

void MAVTracker::publish_ParticleMarker() {
    ZoneScoped;
    pub_ParticleMarker.publish(pfMAV.getParticleMarker());
}

void MAVTracker::publish_BoundingBox() {
    ZoneScoped;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.type=visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = filter_BBX_min[0] + (filter_BBX_max[0] - filter_BBX_min[0]) / 2;
    marker.pose.position.y = filter_BBX_min[1] + (filter_BBX_max[1] - filter_BBX_min[1]) / 2;
    marker.pose.position.z = filter_BBX_min[2] + (filter_BBX_max[2] - filter_BBX_min[2]) / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = filter_BBX_max[0] - filter_BBX_min[0];
    marker.scale.y = filter_BBX_max[1] - filter_BBX_min[1];
    marker.scale.z = filter_BBX_max[2] - filter_BBX_min[2];
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;
    marker.lifetime = ros::Duration();
    
    pub_FilterBBX.publish(marker);
}

void MAVTracker::execute(){
    ZoneScoped;
    // Main execution
    // The tracker runs with set frequency

    if (tracking_type == ParticleFilter) {
        //predict particles
        // pretty much add a random number from the normal distribution with 
        // 0 mean and predict_std std to each particle coordinate along each axle
        pfMAV.particles_predict();
    // only start prediction if the first measurement has been received
    } else if (tracking_type == EKF && !kfMAV.is_first_measurement()) {
        kfMAV.predictEKF(1.0/frequency);
    }

    
    // do update step if new measurement has arrived
    if (new_measurement) {
        ZoneText("update_runs", strlen("update_runs"));
        // Particle filter
        if (tracking_type == ParticleFilter) {
            // update weights and resample
            // find nearest measurement point. Update weight based on distance to nearest point than normalize weights
            pfMAV.particles_update(cloud_mav);
            new_measurement = false;
        } else if (tracking_type == EKF) {
            // update the Kalman filter with the new measurement
            // check if its the first measurement, if so, set the first measurement
            if (kfMAV.is_first_measurement()) {
                kfMAV.set_first_measurement(cloud_mav);
                /// TODO: check if update should be called here
                new_measurement = false;
                return;
            }
            kfMAV.update(cloud_mav);
            new_measurement = false;
        }
    }

    if (tracking_type == ParticleFilter) {
        // get new estimate
        // calculates the average of the particle positions
        mav_velocity_marker = pfMAV.getEstimate();
    } else if (tracking_type == EKF) {
        // get new estimate
        mav_velocity_marker = kfMAV.getEstimate();
    }

    bool is_tracked = false;
    if (tracking_type == ParticleFilter) {
        is_tracked = pfMAV.get_is_tracked();
    } else if (tracking_type == EKF) {
        is_tracked = kfMAV.get_is_tracked();
    }
    publish_MAV(is_tracked);

    if (tracking_type == ParticleFilter) {
        publish_ParticleMarker();
    }
    
    if (filter_BBX){
        publish_BoundingBox();
    }
}

void MAVTracker::loop() {
    // Main loop
    while (nh.ok()) {
        // Check for new messages
        ros::spinOnce();

        // execute one loop
        execute();

        // sleep to match pre-defined frequency
        rate->sleep();
    }
}

int main(int argc, char **argv) {   
    ros::init(argc,argv,"mav_track_node");

    MAVTracker mav_track;

    mav_track.loop();

    return 0;
}