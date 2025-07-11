// Includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// tf and math
#include <tf/transform_listener.h>
#include <cmath>
#include <unordered_map>

// Constants and Macros
#define WGS84_A 6378137.0
#define WGS84_E 8.1819190842622e-2
#define DEG2RAD (M_PI / 180.0)

// ===================== Configuration =====================
enum class ConfigType { BUDATETENY_2025_03_11, BUDATETENY_2025_03_20, BARACSKA_2025_04_02 };

struct Config {
    double lat, lon, alt, heading;
};

const std::unordered_map<ConfigType, Config> CONFIGS = {
    {ConfigType::BUDATETENY_2025_03_11, {47.391927304, 18.989434494, 99.63, 143.05}},
    {ConfigType::BUDATETENY_2025_03_20, {47.298701604, 18.751286286535, 100.0, 304.0}},
    {ConfigType::BARACSKA_2025_04_02, {47.298697429, 18.751290692, 101.84, 302.36}}
};

ConfigType ACTIVE_CONFIG = ConfigType::BARACSKA_2025_04_02;
const Config& turret = CONFIGS.at(ACTIVE_CONFIG);

// ===================== Globals =====================
sensor_msgs::NavSatFix mav_geo;
visualization_msgs::Marker gps_marker;
ros::Publisher marker_pub;

// ===================== Utility Functions =====================
void computeLocalCoordinates(const sensor_msgs::NavSatFix& mav,
                             const Config& turret,
                             double heading, double pitch, double roll) {
    auto toECEF = [](double lat, double lon, double alt, double& x, double& y, double& z) {
        lat *= DEG2RAD; lon *= DEG2RAD;
        double clat = cos(lat), slat = sin(lat);
        double clon = cos(lon), slon = sin(lon);
        double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);
        x = (N + alt) * clat * clon;
        y = (N + alt) * clat * slon;
        z = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;
    };

    // ECEF conversion
    double mx, my, mz, tx, ty, tz;
    toECEF(mav.latitude, mav.longitude, mav.altitude, mx, my, mz);
    toECEF(turret.lat, turret.lon, turret.alt, tx, ty, tz);

    // ENU
    double dx = mx - tx, dy = my - ty, dz = mz - tz;
    double lat = turret.lat * DEG2RAD, lon = turret.lon * DEG2RAD;
    double slat = sin(lat), clat = cos(lat), slon = sin(lon), clon = cos(lon);
    double enu_e = -slon * dx + clon * dy;
    double enu_n = -slat * clon * dx - slat * slon * dy + clat * dz;
    double enu_u = clat * clon * dx + clat * slon * dy + slat * dz;

    // Rotation
    heading *= DEG2RAD;
    pitch = -pitch * DEG2RAD; // nose-down convention
    roll *= DEG2RAD;

    double ch = cos(heading), sh = sin(heading);
    double cp = cos(pitch), sp = sin(pitch);
    double cr = cos(roll), sr = sin(roll);

    double R[3][3] = {
        {ch * cp, ch * sp * sr - sh * cr, ch * sp * cr + sh * sr},
        {sh * cp, sh * sp * sr + ch * cr, sh * sp * cr - ch * sr},
        {-sp, cp * sr, cp * cr}
    };

    // Transform
    double x = R[0][0] * enu_n + R[1][0] * enu_e + R[2][0] * enu_u;
    double y = -(R[0][1] * enu_n + R[1][1] * enu_e + R[2][1] * enu_u);
    double z = R[0][2] * enu_n + R[1][2] * enu_e + R[2][2] * enu_u;

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    gps_marker.points.push_back(p);
    gps_marker.header.stamp = mav_geo.header.stamp;
    marker_pub.publish(gps_marker);
}

// ===================== ROS Callbacks =====================
void geoCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    mav_geo = *msg;
    computeLocalCoordinates(mav_geo, turret, turret.heading, 0, 0);
}

// ===================== Main =====================
int main(int argc, char** argv) {
    ros::init(argc, argv, "initial_heading_node");
    ros::NodeHandle nh("~");

    ros::Subscriber geo_sub = nh.subscribe("/geo_coordinates", 1, geoCallback);
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("/gps_marker", 1);

    gps_marker.header.frame_id = "world";
    //gps_marker.header.stamp = ros::Time::now();
    gps_marker.ns = "gps_marker";
    gps_marker.id = 0;
    gps_marker.type = visualization_msgs::Marker::LINE_STRIP;
    gps_marker.action = visualization_msgs::Marker::ADD;
    gps_marker.pose.position.x = 0.0;
    gps_marker.pose.position.y = 0.0;
    gps_marker.pose.position.z = 0.0;
    gps_marker.pose.orientation.x = 0.0;
    gps_marker.pose.orientation.y = 0.0;
    gps_marker.pose.orientation.z = 0.0;
    gps_marker.pose.orientation.w = 1.0;
    gps_marker.scale.x = 0.3;
    gps_marker.scale.y = 0.5;
    gps_marker.scale.z = 0.5;
    gps_marker.color.a = 1.0;
    gps_marker.color.r = 0.0;
    gps_marker.color.g = 1.0;
    gps_marker.color.b = 0.0;

    ros::Rate rate(50.0);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
