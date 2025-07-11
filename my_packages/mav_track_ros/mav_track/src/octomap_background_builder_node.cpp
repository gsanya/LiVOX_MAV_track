#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/SetBool.h>

// Tracy
#include "tracy/Tracy.hpp"

#include "mav_track/BuildOctoMapBackground.h"  // This is auto-generated!


// This Node subscribes to a PointCloud2 topic and when the build_octomap_background
// service is called, it builds an Octomap background model from the received point clouds.
// It publishes the Octomap to a topic, so we can visualize the build process and it also 
// returns it as a service response.


class OctomapBackgroundBuilder {
  private:
    // ROS NodeHandle
    ros::NodeHandle nh;

    // Params
    std::string topic_LidarCloud;
    std::string topic_BgModel;
    std::string frame;
    std::string service_ScanningMode;
    double octomap_resolution;
    bool octomap_inflate;
    bool filter_distance_min;
    double filter_distance_min_value;
    bool filter_BBX;
    std::vector<double> filter_BBX_min;
    std::vector<double> filter_BBX_max;
    std::string service_BuildOctoMap_name;
    ros::Rate* rate;
    double frequency;
    
    // Variables
    std::deque<sensor_msgs::PointCloud2::ConstPtr> pointcloud_buffer;
    std::mutex pointcloud_mutex;

    bool bg_model_building;
    std::mutex build_mutex;

    octomap::OcTree* bg_model;

    // Services, subscribers, and publishers
    tf::TransformListener transform_listener;
    ros::Subscriber sub_LidarCloud;
    ros::Publisher pub_BgModel;
    ros::ServiceServer service_BuildOctoMap;
    ros::ServiceClient srv_ScanningMode;

  public:

    // Constructor
    OctomapBackgroundBuilder();

    void callback_LidarCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void publish_BgModel();

    bool buildOctoMapCallback(mav_track::BuildOctoMapBackground::Request &req,
                        mav_track::BuildOctoMapBackground::Response &res);

    bool call_service_ScanningMode(bool data);

    void inflateBgModel();

};

OctomapBackgroundBuilder::OctomapBackgroundBuilder() : nh("~") {
    // Initialize parameters
    topic_LidarCloud = "/livox/lidar";
    topic_BgModel = "/mav_track/bgmodel";
    frame = "world";
    service_ScanningMode = "/turret_cmd/scanning_mode";
    octomap_resolution = 0.1;
    octomap_inflate = true;
    filter_distance_min = true;
    filter_distance_min_value = 1.0;
    filter_BBX = true;
    filter_BBX_min.assign(3, -10.0);
    filter_BBX_max.assign(3, 10.0);
    service_BuildOctoMap_name = "/mav_track/build_octomap_background";
    frequency = 10.0;

    nh.getParam("Frequency", frequency);
    nh.getParam("Frame", frame);
    nh.getParam("Topic_Sub_LidarCloud", topic_LidarCloud);
    nh.getParam("Topic_Pub_BgModel", topic_BgModel);  
    nh.getParam("Service_ScanningMode", service_ScanningMode);
    nh.getParam("Octomap_Resolution", octomap_resolution);
    nh.getParam("Octomap_Inflate", octomap_inflate);
    nh.getParam("Filter_Distance_Min", filter_distance_min);
    nh.getParam("Filter_Distance_Min_Value", filter_distance_min_value);
    nh.getParam("Filter_BBX", filter_BBX);
    nh.getParam("Filter_BBX_min", filter_BBX_min);
    nh.getParam("Filter_BBX_max", filter_BBX_max);
    nh.getParam("Service_BuildOctoMap_name", service_BuildOctoMap_name);  
    

    // print params
    ROS_INFO_STREAM("OctomapBackgroundBuilder Params:");
    ROS_INFO_STREAM("  Frequency: " << frequency);
    ROS_INFO_STREAM("  Frame: " << frame);
    ROS_INFO_STREAM("  Topic_Sub_LidarCloud: " << topic_LidarCloud);
    ROS_INFO_STREAM("  Topic_Pub_BgModel: " << topic_BgModel);
    ROS_INFO_STREAM("  Service_ScanningMode: " << service_ScanningMode);
    ROS_INFO_STREAM("  Octomap_Resolution: " << octomap_resolution);
    ROS_INFO_STREAM("  Octomap_Inflate: " << octomap_inflate);
    ROS_INFO_STREAM("  Filter_Distance_Min: " << filter_distance_min);
    ROS_INFO_STREAM("  Filter_Distance_Min_Value: " << filter_distance_min_value);
    ROS_INFO_STREAM("  Filter_BBX: " << filter_BBX);
    ROS_INFO_STREAM("  Filter_BBX_min: [" 
        << filter_BBX_min[0] << ", "
        << filter_BBX_min[1] << ", "
        << filter_BBX_min[2] << "]");
    ROS_INFO_STREAM("  Filter_BBX_max: [" 
        << filter_BBX_max[0] << ", "
        << filter_BBX_max[1] << ", "
        << filter_BBX_max[2] << "]");
    ROS_INFO_STREAM("  Service_BuildOctoMap_name: " << service_BuildOctoMap_name);

    // we spin() this, not SpinOnce()!! rate is only used in the service callback
    rate = new ros::Rate(frequency);

    int queue_sub = 1;
    int queue_pub = 1;

    // Services, subscribers, and publishers
    sub_LidarCloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_LidarCloud, queue_sub, &OctomapBackgroundBuilder::callback_LidarCloud, this);
    pub_BgModel = nh.advertise<octomap_msgs::Octomap>(topic_BgModel, queue_pub, true);
    service_BuildOctoMap = nh.advertiseService(service_BuildOctoMap_name, &OctomapBackgroundBuilder::buildOctoMapCallback, this);
    srv_ScanningMode = nh.serviceClient<std_srvs::SetBool>(service_ScanningMode);
    
    // Initialize variables
    bg_model_building = false;
    bg_model = new octomap::OcTree(octomap_resolution);
}

void OctomapBackgroundBuilder::publish_BgModel() {
    ZoneScoped;
    octomap_msgs::Octomap message;
    if (!octomap_msgs::fullMapToMsg(*bg_model, message)) {
        ROS_WARN_STREAM("Was not able to create octomap message.");
        return;
    }
    message.header.stamp = ros::Time::now();
    message.header.frame_id = frame;
    pub_BgModel.publish(message);
}

void OctomapBackgroundBuilder::callback_LidarCloud(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ZoneScoped;
    // check if we are building the model
    bool store_pointcloud = false;
    {
        std::lock_guard<std::mutex> lock(build_mutex);
        store_pointcloud = bg_model_building;
    }
    // if we build it, store the pointcloud
    if (store_pointcloud) {
        {
            std::lock_guard<std::mutex> lock(pointcloud_mutex);
            pointcloud_buffer.push_back(msg);
        }
    }
}

bool OctomapBackgroundBuilder::call_service_ScanningMode(bool data) {
    ZoneScoped;
    std_srvs::SetBool srv;
    srv.request.data = data;
    bool success = srv_ScanningMode.call(srv);
    ROS_INFO_STREAM("Called ScanningMode service: '" << srv.response.message << "'");
    return success && srv.response.success;
}

bool OctomapBackgroundBuilder::buildOctoMapCallback(
        mav_track::BuildOctoMapBackground::Request &req,
        mav_track::BuildOctoMapBackground::Response &res) {
    ZoneScoped;
    ROS_INFO("Building OctoMap from %d frames with resolution %.2f", req.num_frames, octomap_resolution);

    // tell turret to start scanning
    call_service_ScanningMode(true);
    // turn on flag
    {
        std::lock_guard<std::mutex> lock(build_mutex);
        bg_model_building = true;
    }

    // Build the octomap
    int frames_added = 0;
    while (frames_added < req.num_frames) {
        // copy pointcloud buffer
        std::deque<sensor_msgs::PointCloud2::ConstPtr> buffer_copy;
        {
            std::lock_guard<std::mutex> lock(pointcloud_mutex);
            buffer_copy = pointcloud_buffer;
        }
        // if new pointclouds are available
        if (buffer_copy.size() > frames_added) {
            while (frames_added < buffer_copy.size()) {
                sensor_msgs::PointCloud2::ConstPtr msg = buffer_copy[frames_added];

                if (!transform_listener.waitForTransform(frame, msg->header.frame_id, msg->header.stamp, ros::Duration(2.0))) {
                    ROS_WARN_STREAM("Was not able to lookup transform from [" << msg->header.frame_id << "] to [" << frame << "].");
                    // // turn off flag
                    // {
                    //     std::lock_guard<std::mutex> lock(build_mutex);
                    //     bg_model_building = false;
                    //     ROS_INFO("bg_model_building turned off: %d", bg_model_building);
                    // }
                    // empty the buffer
                    {
                        std::lock_guard<std::mutex> lock(pointcloud_mutex);
                        pointcloud_buffer.clear();
                        ROS_INFO("pointcloud_buffer cleared: %d", pointcloud_buffer.size());
                    }

                    
                    continue;
                }
       
                octomap::Pointcloud opcl;
                octomap::pointCloud2ToOctomap(*msg, opcl);

                // filter: remove near points
                if (filter_distance_min)
                    opcl.minDist(filter_distance_min_value);

                // transform pointcloud into world frame
                tf::StampedTransform transform_tf;
                transform_listener.lookupTransform(frame, msg->header.frame_id, msg->header.stamp, transform_tf);
                tf::Pose poseTf(transform_tf.getRotation(), transform_tf.getOrigin());
                opcl.transform(octomap::poseTfToOctomap(poseTf));

                // filter: remove ground plane, ceiling, or far away points
                if (filter_BBX) {
                    octomap::point3d bbx_min(filter_BBX_min[0], filter_BBX_min[1], filter_BBX_min[2]);
                    octomap::point3d bbx_max(filter_BBX_max[0], filter_BBX_max[1], filter_BBX_max[2]);
                    opcl.crop(bbx_min, bbx_max);
                }
                // get sensor origin in global frame
                geometry_msgs::PointStamped origin_lidar;
                origin_lidar.header = msg->header;
                origin_lidar.point.x = origin_lidar.point.y = origin_lidar.point.z = 0.0;
                transform_listener.transformPoint(frame, origin_lidar, origin_lidar);
                // insert pointcloud into background model
                bg_model->insertPointCloud(opcl, octomap::point3d(origin_lidar.point.x, origin_lidar.point.y, origin_lidar.point.z));
                frames_added++;
            }
            publish_BgModel();
        } else {
            //ROS_INFO("No new pointclouds available, waiting...");
            rate->sleep();
        }
    }

    // stop turret scanning
    call_service_ScanningMode(false);
    // turn off flag
    {
        std::lock_guard<std::mutex> lock(build_mutex);
        bg_model_building = false;
    }
    // empty the buffer
    {
        std::lock_guard<std::mutex> lock(pointcloud_mutex);
        pointcloud_buffer.clear();
    }
    
    if (octomap_inflate)
        inflateBgModel();

    publish_BgModel();

    // Convert octomap to message and assign to response
    octomap_msgs::Octomap map_msg;
    if (!octomap_msgs::fullMapToMsg(*bg_model, map_msg)) {
        ROS_WARN_STREAM("Was not able to create octomap message for response.");
        return false;
    }
    res.map = map_msg;

    bg_model->clear();

    return true;
}

void OctomapBackgroundBuilder::inflateBgModel() {
    ZoneScoped;
    // inflate the background model by one voxel in every direction
    // see https://github.com/OctoMap/octomap/issues/42
    octomap::OcTree* bg_model_inflated = new octomap::OcTree(*bg_model);
    // see https://octomap.github.io/octomap/doc/classleaf__iterator.html
    for (octomap::OcTree::leaf_iterator it = bg_model->begin_leafs(), end = bg_model->end_leafs(); it != end; ++it)
    {
        // get node
        octomap::OcTreeKey okey = it.getKey();
        octomap::OcTreeNode* onode = bg_model->search(okey);
        // go through every neighbouring node
        for (int z : {-1, 0, 1}) for (int y : {-1, 0, 1}) for (int x : {-1, 0, 1})
        {
            if (z == 0 && y == 0 && x == 0) continue;
            octomap::OcTreeKey okey_n = okey;
            okey_n[0] += x; okey_n[1] += y; okey_n[2] += z;
            octomap::OcTreeNode* onode_n = bg_model_inflated->search(okey_n);
            if (onode_n == NULL)
            {
                // create node (setting initial occupancy not important because will be overwritten afterwards)
                onode_n = bg_model_inflated->updateNode(okey_n, true);
                // set node value to center node value
                onode_n = bg_model_inflated->setNodeValue(okey_n, onode->getValue());
            }
            else if (onode->getValue() > onode_n->getValue())
            {
                // set node value to center node value if center node value is bigger than current own value
                onode_n = bg_model_inflated->setNodeValue(okey_n, onode->getValue());
            }
        }
    }
    // delete old bg model
    bg_model->clear();
    delete bg_model;
    bg_model = NULL;
    // create new bg model from inflated bg model
    bg_model = new octomap::OcTree(*bg_model_inflated);
    // delete inflated bg model
    delete bg_model_inflated;
    bg_model_inflated = NULL;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_background_builder_node");

    OctomapBackgroundBuilder octomap_background_builder;
    
    ROS_INFO("OctoMap background builder service is ready.");
    ros::MultiThreadedSpinner spinner(2); // 2 threads spinner.
    spinner.spin(); 
    // ros::spin();
    return 0;
}