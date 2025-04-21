#pragma once

#include <filesystem>
#include <fstream>

#include "Eigen/Geometry"
#include "nlohmann/json.hpp"
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

    LogWriter(const std::string &directory_path)
        : frame_count_(0)
    {
        std::filesystem::path dir_path(directory_path);
        if (!std::filesystem::exists(dir_path))
        {
            std::filesystem::create_directories(dir_path);
        }
        dir_path_ = directory_path;
        is_initialized_ = false;
    }

    void write(const sensor_msgs::msg::Image::ConstSharedPtr &image, const Metadata &metadata)
    {
        cv::Mat cv_image;
        try
        {
            if (image->encoding == "bgr8")
            {
                cv_image = cv::Mat(image->height, image->width, CV_8UC3,
                                   const_cast<unsigned char *>(image->data.data()), image->step);
            }
            else if (image->encoding == "rgb8")
            {
                cv_image = cv::Mat(image->height, image->width, CV_8UC3,
                                   const_cast<unsigned char *>(image->data.data()), image->step);
                cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
            }
            else if (image->encoding == "mono8")
            {
                cv_image = cv::Mat(image->height, image->width, CV_8UC1,
                                   const_cast<unsigned char *>(image->data.data()), image->step);
                cv::cvtColor(cv_image, cv_image, cv::COLOR_GRAY2BGR);
            }
            else
            {
                throw std::runtime_error("Unsupported image encoding: " + image->encoding);
            }

            if (!is_initialized_)
            {
                int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPG codec
                double fps = 30.0;
                video_writer_.open((dir_path_ / "video.avi").string(), fourcc, fps, cv_image.size(), true);
                if (!video_writer_.isOpened())
                {
                    throw std::runtime_error("Could not open video file for writing");
                }
                is_initialized_ = true;
            }

            video_writer_.write(cv_image);

            nlohmann::json json_data;
            json_data["epoch_nanos"] = metadata.epoch_nanos;
            json_data["latitude"] = metadata.latitude;
            json_data["longitude"] = metadata.longitude;
            json_data["altitude"] = metadata.altitude;
            json_data["orientation"] = {
                {"qw", metadata.qw}, {"qx", metadata.qx}, {"qy", metadata.qy}, {"qz", metadata.qz}};

            std::string json_filename = (dir_path_ / (std::to_string(frame_count_) + ".json")).string();
            std::ofstream json_file(json_filename);
            json_file << json_data.dump(2);

            frame_count_++;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in LogWriter::write: " << e.what() << std::endl;
        }
    }

    ~LogWriter()
    {
        if (video_writer_.isOpened())
        {
            video_writer_.release();
        }
    }

private:
    std::filesystem::path dir_path_;
    cv::VideoWriter video_writer_;
    int frame_count_;
    bool is_initialized_;
};
