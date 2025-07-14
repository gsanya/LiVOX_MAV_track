#include <pcl/kdtree/kdtree_flann.h>
#include <tf2/LinearMath/Vector3.h>

// Tracy
#include "tracy/Tracy.hpp"

#include "mav_track/mav_track_particlefilter.hpp"

MAVTrackParticle::MAVTrackParticle() {
    // position (state of particle)
    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;
    last_position.x = position.x;
    last_position.y = position.y;
    last_position.z = position.z;

    // velocity (state of particle)
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;
    acceleration.x = 0.0;
    acceleration.y = 0.0;
    acceleration.z = 0.0;
    // weight of particle
    weight = 0.0;
}

MAVTrackParticle::MAVTrackParticle(geometry_msgs::Point position,geometry_msgs::Point velocity, double weight) {
    this->position = position;
    this->velocity = velocity;
    this->weight = weight;
}

MAVTrackParticleFilter::MAVTrackParticleFilter() {
    // Parameters
    frame = "world";
    Nparticles = 1000;
    initial_sampling_zone_radius = 5.0;
    initial_sampling_zone_height = 2.0;
    predict_std_pos = 0.1;
    predict_std_vel = 0.1;
    predict_std_acc = 0.1;
    sensing_std = 0.5;
    frequency = 10.0;
    delta_time = 1.0 / frequency;
    av_std_pos = 1000000000.0; // very high value to start with, so that the first estimate is not considered tracked
    // position_estimate_history_size = 10;

    // Initializations
    markers_initialized = false;
    position_estimate.x = 0.0;
    position_estimate.y = 0.0;
    position_estimate.z = 0.0;
    
    last_position_estimate.x = 0.0;
    last_position_estimate.y = 0.0;
    last_position_estimate.z = 0.0;

    // position_estimate_history.reserve(position_estimate_history_size);

    // Needed?
    velocity_estimate.x = 0.0;
    velocity_estimate.y = 0.0;
    velocity_estimate.z = 0.0;
    
    // Marker
    mav_pose_marker.header.frame_id = frame;
    mav_pose_marker.type = visualization_msgs::Marker::ARROW;
    mav_pose_marker.action = visualization_msgs::Marker::ADD;

    mav_pose_marker.points.resize(2);
    mav_pose_marker.points[0].x = 0.0;
    mav_pose_marker.points[0].y = 0.0;
    mav_pose_marker.points[0].z = 0.0;
    mav_pose_marker.points[1].x = 1.0;
    mav_pose_marker.points[1].y = 1.0;
    mav_pose_marker.points[1].z = 1.0;
    mav_pose_marker.scale.x = 0.1;
    mav_pose_marker.scale.y = 0.1;
    mav_pose_marker.scale.z = 0.1;
    mav_pose_marker.color.r = 1.0;
    mav_pose_marker.color.a = 0.0;

    // Do not forget to call particles_initialize()
    particles_initialized = false;

    motion_model_type = OnlyPosition;
    velocity_when_not_tracked = false;
}

MAVTrackParticleFilter::~MAVTrackParticleFilter() {
    // nothing yet
}

void MAVTrackParticleFilter::set_motion_model_type(
        PFMotionModelType motion_model_type,
        bool velocity_when_not_tracked,
        bool acceleration_not_tracked) {
    ZoneScoped;
    this->motion_model_type = motion_model_type;
    this->velocity_when_not_tracked = velocity_when_not_tracked;
    this->acceleration_not_tracked = acceleration_not_tracked;
}

void MAVTrackParticleFilter::set_Nparticles(int Nparticles) {
    ZoneScoped;
    this->Nparticles = Nparticles;
    particles_initialized = false;
}

void MAVTrackParticleFilter::set_initial_sampling_zone(double radius, double height, double x, double y, double z) {
    ZoneScoped;
    this->initial_sampling_zone_radius = radius;
    this->initial_sampling_zone_height = height;
    this->initial_sampling_zone_x = x;
    this->initial_sampling_zone_y = y;
    this->initial_sampling_zone_z = z;
    particles_initialized = false;
}

void MAVTrackParticleFilter::set_default_predict_std(double std_pos, double std_vel, double std_acc) {
    ZoneScoped;
    this->predict_std_pos = std_pos;
    this->predict_std_vel = std_vel;
    this->predict_std_acc = std_acc;
}

void MAVTrackParticleFilter::set_default_sensing_std(double std) {
    ZoneScoped;
    this->sensing_std = std;
}

void MAVTrackParticleFilter::set_frame(std::string frame) {
    ZoneScoped;
    this->frame = frame;
}

void MAVTrackParticleFilter::set_frequency(double frequency) {
    ZoneScoped;
    this->frequency = frequency;
    this->delta_time = 1.0 / frequency;
}

void MAVTrackParticleFilter::particles_initialize() {
    ZoneScoped;
    // initialize particles and weights
    weights.clear();
    particle.clear();
    particle.reserve(Nparticles);
    for (int i = 0; i < Nparticles; i++) particle.emplace_back();
    // sample particles within initial sampling zone
    std::random_device rd; // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> disAngle(0.0, 2.0*M_PI);
    std::uniform_real_distribution<float> disR(0.0, initial_sampling_zone_radius);
    std::uniform_real_distribution<float> disZ(0.0, initial_sampling_zone_height);
    for (int i = 0; i < particle.size(); i++) {
        float angle = disAngle(gen);
        float radius = disR(gen);
        particle[i].position.x = radius*sin(angle) + initial_sampling_zone_x;
        particle[i].position.y = radius*cos(angle) + initial_sampling_zone_y;
        particle[i].position.z = disZ(gen) + initial_sampling_zone_z;
        particle[i].velocity.x = 0.0;
        particle[i].velocity.y = 0.0;
        particle[i].velocity.z = 0.0;
        particle[i].acceleration.x = 0.0;
        particle[i].acceleration.y = 0.0;
        particle[i].acceleration.z = 0.0;
        // set weight
        particle[i].weight = 1.0/particle.size();
        weights.emplace_back(particle[i].weight);
    }
    particle_timestamp = ros::Time::now();
    particles_initialized = true;
}

void MAVTrackParticleFilter::init_markers() {
    ZoneScoped;
    // Initialize visualization markers. Will get called automatically when needed.
    particle_marker.header.frame_id = frame;
    particle_marker.ns = "particles";
    particle_marker.id = 0;
    particle_marker.type = visualization_msgs::Marker::POINTS;
    particle_marker.action = visualization_msgs::Marker::ADD;
    particle_marker.pose.position.x = 0.0;
    particle_marker.pose.position.y = 0.0;
    particle_marker.pose.position.z = 0.0;
    particle_marker.pose.orientation.x = 0.0;
    particle_marker.pose.orientation.y = 0.0;
    particle_marker.pose.orientation.z = 0.0;
    particle_marker.pose.orientation.w = 1.0;
    double particle_marker_size = 0.05;
    particle_marker.scale.x = particle_marker_size;
    particle_marker.scale.y = particle_marker_size;
    particle_marker.scale.z = particle_marker_size; // (scale.z not used in POINTS)
    particle_marker.color.r = 1.0;
    particle_marker.color.g = 1.0;
    particle_marker.color.b = 0.0;
    particle_marker.color.a = 1.0;
    particle_marker.lifetime = ros::Duration();
    // Points
    particle_marker.points.clear();
    for (int i = 0; i < particle.size(); i++)
    {
        particle_marker.points.emplace_back(particle[i].position);
    }
    markers_initialized = true;
}

visualization_msgs::Marker MAVTrackParticleFilter::getParticleMarker() {
    ZoneScoped;
    if (!markers_initialized) {
        init_markers();
    } else {
        for (int i = 0; i < particle_marker.points.size(); i++) {
            particle_marker.points[i] = particle[i].position;
        }
    }
    particle_marker.header.stamp = particle_timestamp;
    return particle_marker;
}

void MAVTrackParticleFilter::particles_predict() {
    ZoneScoped;
    // Predict step: apply noise to account for uncertainty in system state (no process or motion model except that)
    std::random_device rd;
    std::mt19937 gen(rd());
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<float> d_pos(0.0, predict_std_pos);
    std::normal_distribution<float> d_vel(0.0, predict_std_vel);
    std::normal_distribution<float> d_acc(0.0, predict_std_acc);
    // iterate through particles
    for (int i = 0; i < particle.size(); i++) {     
        // predict particle params
        if (is_tracked || velocity_when_not_tracked) {
            float v, yaw, yaw_rate, ratio;
            switch (motion_model_type) {
                case Delta:
                    //the predict step runs with constant frequency, so delta_t simplifies out
                    particle[i].position.x += d_pos(gen) + particle[i].velocity.x * delta_time;
                    particle[i].position.y += d_pos(gen) + particle[i].velocity.y * delta_time;
                    particle[i].position.z += d_pos(gen) + particle[i].velocity.z * delta_time;
                    ratio = is_tracked ? 1.0f : (predict_std_pos / av_std_pos);
                    particle[i].velocity.x = ratio * (particle[i].position.x - particle[i].last_position.x) / delta_time;
                    particle[i].velocity.y = ratio * (particle[i].position.y - particle[i].last_position.y) / delta_time;
                    particle[i].velocity.z = ratio * (particle[i].position.z - particle[i].last_position.z) / delta_time;
                    particle[i].acceleration.x = 0.0;
                    particle[i].acceleration.y = 0.0;
                    particle[i].acceleration.z = 0.0;
                    break;
                case ConstVelocity:
                    particle[i].position.x += d_pos(gen) + particle[i].velocity.x * delta_time;
                    particle[i].position.y += d_pos(gen) + particle[i].velocity.y * delta_time;
                    particle[i].position.z += d_pos(gen) + particle[i].velocity.z * delta_time;
                    particle[i].velocity.x += d_vel(gen);
                    particle[i].velocity.y += d_vel(gen);
                    particle[i].velocity.z += d_vel(gen);
                    particle[i].acceleration.x = 0.0;
                    particle[i].acceleration.y = 0.0;
                    particle[i].acceleration.z = 0.0;
                    break;
                case ConstAcceleration:
                    particle[i].position.x += d_pos(gen) + particle[i].velocity.x * delta_time + 0.5 * particle[i].acceleration.x * delta_time * delta_time;
                    particle[i].position.y += d_pos(gen) + particle[i].velocity.y * delta_time + 0.5 * particle[i].acceleration.y * delta_time * delta_time;;
                    particle[i].position.z += d_pos(gen) + particle[i].velocity.z * delta_time + 0.5 * particle[i].acceleration.z * delta_time * delta_time;;
                    particle[i].velocity.x += d_vel(gen) + particle[i].acceleration.x * delta_time;
                    particle[i].velocity.y += d_vel(gen) + particle[i].acceleration.y * delta_time;
                    particle[i].velocity.z += d_vel(gen) + particle[i].acceleration.z * delta_time;
                    if (is_tracked || acceleration_not_tracked) {
                        particle[i].acceleration.x = d_acc(gen);
                        particle[i].acceleration.y = d_acc(gen);
                        particle[i].acceleration.z = d_acc(gen);
                    } else {
                        particle[i].acceleration.x = 0.0;
                        particle[i].acceleration.y = 0.0;
                        particle[i].acceleration.z = 0.0;
                    }
                    break;
                case ConstantTurnRateWithZVelocity:
                    // In this case the variables for the particles are not as
                    // their name suggests. They are:
                    // particle[i].position.x,y,z: position
                    // particle[i].velocity.x: velocity in the xy plane (v)
                    // particle[i].velocity.y: angle of the velocity vector (yaw)
                    // particle[i].velocity.z: velocity in the z axis (v_z)
                    // particle[i].acceleration.x: turn rate around z axis (yaw_rate)
                    // similarly to the EKF, we use the same noise params for
                    // v, v_z and yaw, which is d_vel(gen)
                    // and we use d_acc(gen) for yaw_rate
                    v = particle[i].velocity.x;
                    yaw = particle[i].velocity.y;
                    yaw_rate = particle[i].acceleration.x;
                    
                    // position update
                    // avoid division by zero
                    if (fabs(yaw_rate) > 0.001) {
                        particle[i].position.x += d_pos(gen) + v / yaw_rate * (sin(yaw + yaw_rate * delta_time) - sin(yaw));
                        particle[i].position.y += d_pos(gen) + v / yaw_rate * (cos(yaw) - cos(yaw + yaw_rate * delta_time));
                    } else {
                        particle[i].position.x += d_pos(gen) + v * delta_time * cos(yaw);
                        particle[i].position.y += d_pos(gen) + v * delta_time * sin(yaw);
                        yaw_rate = 0.001;
                    }
                    particle[i].position.z += d_pos(gen) + particle[i].velocity.z * delta_time;
                    // v update
                    particle[i].velocity.x += d_vel(gen);
                    // yaw update
                    particle[i].velocity.y += d_vel(gen) + yaw_rate * delta_time;
                    // v_z update
                    particle[i].velocity.z += d_vel(gen);
                    // yaw_rate update
                    particle[i].acceleration.x = yaw_rate + d_acc(gen);

                    // fill others with 0
                    particle[i].acceleration.y = 0.0;
                    particle[i].acceleration.z = 0.0;
                    break;
                case OnlyPosition:
                default:
                    particle[i].position.x += d_pos(gen);
                    particle[i].position.y += d_pos(gen);
                    particle[i].position.z += d_pos(gen);
                    particle[i].velocity.x = 0.0;
                    particle[i].velocity.y = 0.0;
                    particle[i].velocity.z = 0.0;
                    particle[i].acceleration.x = 0.0;
                    particle[i].acceleration.y = 0.0;
                    particle[i].acceleration.z = 0.0;
                    break;
            }        
        }
        // update last position
        particle[i].last_position.x = particle[i].position.x;
        particle[i].last_position.y = particle[i].position.y;
        particle[i].last_position.z = particle[i].position.z;
    }
    // update timestamp
    particle_timestamp = ros::Time::now();
}

float MAVTrackParticleFilter::normal_pdf(const float& mean, const float& std, const float& x) {
    ZoneScoped;
    // Source: https://stackoverflow.com/questions/10847007/using-the-gaussian-probability-density-function-in-c
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - mean) / std;
    return inv_sqrt_2pi / std * std::exp(-0.5f * a * a);
}

void MAVTrackParticleFilter::particles_update(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    ZoneScoped;
    // Update step: update weights using measurements
    double weight_sum = 0.0;

    // K nearest neighbor search with PDF weight assignment
    int K = 1;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for (int i = 0; i < particle.size(); i++) {
        searchPoint.x = particle[i].position.x;
        searchPoint.y = particle[i].position.y;
        searchPoint.z = particle[i].position.z;
        double dist = 0.0;
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (int j = 0; j < pointIdxNKNSearch.size(); ++j) dist += pointNKNSquaredDistance[j];
        }
        dist = sqrt(dist + 0.0001);
        double w = normal_pdf(0.0, sensing_std, dist) + 0.0001;
        particle[i].weight = w;
        weight_sum += particle[i].weight;
    }

    // normalize
    if (weight_sum == 0.0) weight_sum = 1.0;
    for (int i = 0; i < particle.size(); i++) {
        particle[i].weight /= weight_sum;
        weights[i] = particle[i].weight;
    }

    // particles_estimate_usebestweight();
    // OR directly
    particles_resample();
}

void MAVTrackParticleFilter::particles_estimate_usemean() {   
    ZoneScoped;
    // calculate the mean of particle positions as estimate
    double x = 0.0, y = 0.0, z = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double av_x = 0.0, av_y = 0.0, av_z = 0.0;
    double av_vx = 0.0, av_vy = 0.0, av_vz = 0.0;
    for (int i = 0; i < particle.size(); i++) {
        x += particle[i].position.x;
        y += particle[i].position.y;
        z += particle[i].position.z;
        if (motion_model_type == ConstantTurnRateWithZVelocity) {
            // In this case the variables for the particles are not as
            // their name suggests, and we have to perform a little math
            float v = particle[i].velocity.x;
            float yaw = particle[i].velocity.y;
            float v_z = particle[i].velocity.z;           
            vx += v * cos(yaw);
            vy += v * sin(yaw);
            vz += v_z;
        } else {
            vx += particle[i].velocity.x;
            vy += particle[i].velocity.y;
            vz += particle[i].velocity.z;
        }
    }
    av_x = x / particle.size();
    av_y = y / particle.size();
    av_z = z / particle.size();

    if (motion_model_type != OnlyPosition) {
        av_vx = vx / particle.size();
        av_vy = vy / particle.size();
        av_vz = vz / particle.size();
    } else {
        av_vx = 0.0;
        av_vy = 0.0;
        av_vz = 0.0;
    }
    
    // calculate the standard deviation of particle positions to decide if tracking is lost
    double sum_esquare_x = 0.0, sum_esquare_y = 0.0, sum_esquare_z = 0.0;
    for (int i = 0; i < particle.size(); i++) {
        sum_esquare_x += (particle[i].position.x - av_x) * (particle[i].position.x - av_x);
        sum_esquare_y += (particle[i].position.y - av_y) * (particle[i].position.y - av_y);
        sum_esquare_z += (particle[i].position.z - av_z) * (particle[i].position.z - av_z);
    }
    av_std_pos = (sqrt(sum_esquare_x / particle.size())+sqrt(sum_esquare_y / particle.size())+sqrt(sum_esquare_z / particle.size()))/3;
    
    double threshold;
    if (predict_std_pos < sensing_std){
        threshold = sensing_std * 1.5;
    } else {
        threshold = predict_std_pos * 1.5;
    }
    if (av_std_pos < threshold){
        is_tracked = true;
    } else {
        is_tracked = false;
    }

    // Update position and velocity estimate
    position_estimate.x = av_x;
    position_estimate.y = av_y;
    position_estimate.z = av_z;
    velocity_estimate.x = av_vx;
    velocity_estimate.y = av_vy;
    velocity_estimate.z = av_vz;
    
    // Update last position estimate
    last_position_estimate.x = position_estimate.x;
    last_position_estimate.y = position_estimate.y;
    last_position_estimate.z = position_estimate.z;

    // Time  
    mav_pose_marker.header.stamp = particle_timestamp;

    // Update marker
    mav_pose_marker.points[0].x = position_estimate.x;
    mav_pose_marker.points[0].y = position_estimate.y;
    mav_pose_marker.points[0].z = position_estimate.z;
    mav_pose_marker.points[1].x = position_estimate.x + velocity_estimate.x;
    mav_pose_marker.points[1].y = position_estimate.y + velocity_estimate.y;
    mav_pose_marker.points[1].z = position_estimate.z + velocity_estimate.z;
}

visualization_msgs::Marker MAVTrackParticleFilter::getEstimate() {
    ZoneScoped;
    particles_estimate_usemean();
    return mav_pose_marker;
}

void MAVTrackParticleFilter::particles_resample() {
    ZoneScoped;
    // set distribution of particle weights
    std::discrete_distribution<int> d(weights.begin(), weights.end());
    // initiate new particle vector
    std::vector<MAVTrackParticle> particle_new;
    particle_new.reserve(particle.size());
    // sample random new particles according to weight distribution
    std::random_device rd;
    //this generates a pseudo random number based on an integer input
    std::mt19937 gen(rd());
    for (int i = 0; i < particle.size(); i++) {
        particle_new.emplace_back(particle[d(gen)]);
    }
    particle = particle_new;
}

bool MAVTrackParticleFilter::get_is_tracked() {
    ZoneScoped;
    return is_tracked;
}
