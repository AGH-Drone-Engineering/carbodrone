#pragma once

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/altitude.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class LogWriter
{
public:
    struct Metadata
    {
        Metadata() = delete;

        Metadata(const sensor_msgs::msg::Image::ConstSharedPtr &image_in,
                 const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_in,
                 const mavros_msgs::msg::Altitude::ConstSharedPtr &altitude_in,
                 const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_in)
        {
            epoch_nanos =
                static_cast<long long>(image_in->header.stamp.sec) * 1'000'000'000LL + image_in->header.stamp.nanosec;
            latitude = gps_in->latitude;
            longitude = gps_in->longitude;
            altitude = altitude_in->relative;
            qw = pose_in->pose.orientation.w;
            qx = pose_in->pose.orientation.x;
            qy = pose_in->pose.orientation.y;
            qz = pose_in->pose.orientation.z;
        }

        long long epoch_nanos;
        double latitude;
        double longitude;
        double altitude;
        double qw, qx, qy, qz;
    };

    void write(const sensor_msgs::msg::Image::ConstSharedPtr &image, const Metadata &metadata) {}

private:
};
