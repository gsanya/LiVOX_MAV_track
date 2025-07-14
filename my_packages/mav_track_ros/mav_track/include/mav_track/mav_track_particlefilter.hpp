#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>

#include <random>

enum PFMotionModelType {
  OnlyPosition = 0,
  Delta = 1,
  ConstVelocity = 2,
  ConstAcceleration = 3,
  ConstantTurnRateWithZVelocity = 4
};

class MAVTrackParticle {
  public:
    geometry_msgs::Point position;
    //could be a vector of past positions, then more accurate velocity estimation could be given
    geometry_msgs::Point last_position;
    geometry_msgs::Point velocity;
    geometry_msgs::Point acceleration;
    double weight;

    MAVTrackParticle();
    MAVTrackParticle(geometry_msgs::Point position,geometry_msgs::Point velocity, double weight);
};

class MAVTrackParticleFilter {
  private:

    PFMotionModelType motion_model_type;

    // System
    ros::Time particle_timestamp;
    std::vector<MAVTrackParticle> particle;
    std::vector<double> weights;
    bool particles_initialized;
    visualization_msgs::Marker particle_marker;
    bool markers_initialized;
    bool is_tracked;
    geometry_msgs::Point position_estimate;
    geometry_msgs::Point last_position_estimate;
    // std::vector<geometry_msgs::Point> position_estimate_history;
    // int position_estimate_history_size;
    geometry_msgs::Point velocity_estimate;
    visualization_msgs::Marker mav_pose_marker;
    double av_std_pos;

    // Parameters
    std::string frame;
    int Nparticles;
    float initial_sampling_zone_radius, initial_sampling_zone_height,initial_sampling_zone_x, initial_sampling_zone_y, initial_sampling_zone_z; 
    float predict_std_pos, predict_std_vel, predict_std_acc;
    float sensing_std;
    float frequency, delta_time;
    bool velocity_when_not_tracked;
    bool acceleration_not_tracked;

  public:
    // Constructor and Destructor
    MAVTrackParticleFilter();
    ~MAVTrackParticleFilter();
    // Setters
    void set_motion_model_type(
    	PFMotionModelType motion_model_type,
		  bool velocity_when_not_tracked,
      bool acceleration_not_tracked);
	  void set_Nparticles(int Nparticles);
    void set_initial_sampling_zone(double radius, double height, double x, double y, double z);
    void set_default_predict_std(double std_pos, double std_vel, double std_acc);
    void set_default_sensing_std(double std);
    void set_frame(std::string frame);
    void set_frequency(double frequency);

    bool get_is_tracked();

    // Filter initialization
    void particles_initialize();
    // Markers
    void init_markers();
    visualization_msgs::Marker getParticleMarker();
    // Filter prediction
    void particles_predict();
    //void particles_predict_noise(double scale, double std = (1.0));
    //void particles_predict_shift(double dx, double dy, double dz, double std = (-1.0));
    //void particles_predict_shift_lognormal(double dx, double dy, double dz, double std = (-1.0));
    // Filter update
    float normal_pdf(const float& mean, const float& std, const float& x);
    void particles_update(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    // Filter estimation
    //void particles_estimate_usebestweight();
    void particles_estimate_usemean();
    visualization_msgs::Marker getEstimate();
    // Filter resampling
    void particles_resample();    
};
