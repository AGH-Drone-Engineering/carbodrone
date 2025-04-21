#pragma once

#include <array>
#include <memory>
#include <vector>

#include "geo.hpp"

class MissionPlanner
{
public:
    MissionPlanner(const geo::Point &home)
        : _home(home)
    {
    }

    void plan(const geo::Point &p0, const geo::Point &p1, const geo::Point &p2, const geo::Point &p3, double max_stride)
    {
        std::array<geo::Point, 4> points = {p0, p1, p2, p3};

        int closest_point_index = 0;
        double min_dist = _home.distance(points[0]);
        for (size_t i = 1; i < points.size(); i++)
        {
            double dist = _home.distance(points[i]);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_point_index = i;
            }
        }

        double len_of_side_increasing_index =
            points[closest_point_index % 4].distance(points[(closest_point_index + 1) % 4]);
        double len_of_side_opposite_increasing_index =
            points[(closest_point_index + 2) % 4].distance(points[(closest_point_index + 3) % 4]);

        double len_of_side_decreasing_index =
            points[closest_point_index % 4].distance(points[(closest_point_index + 3) % 4]);
        double len_of_side_opposite_decreasing_index =
            points[(closest_point_index + 2) % 4].distance(points[(closest_point_index + 1) % 4]);

        double max_len_increasing_index = std::max(len_of_side_increasing_index, len_of_side_opposite_increasing_index);
        double max_len_decreasing_index = std::max(len_of_side_decreasing_index, len_of_side_opposite_decreasing_index);

        bool is_increasing_index = max_len_increasing_index < max_len_decreasing_index;
        double len_of_side_to_subdivide =
            is_increasing_index ? len_of_side_increasing_index : len_of_side_decreasing_index;

        int num_subdivisions = std::ceil(len_of_side_to_subdivide / max_stride);

        int side1_start_i, side1_end_i, side2_start_i, side2_end_i;

        if (is_increasing_index)
        {
            side1_start_i = closest_point_index % 4;
            side1_end_i = (closest_point_index + 1) % 4;
            side2_start_i = (closest_point_index + 3) % 4;
            side2_end_i = (closest_point_index + 2) % 4;
        }
        else
        {
            side1_start_i = closest_point_index % 4;
            side1_end_i = (closest_point_index + 3) % 4;
            side2_start_i = (closest_point_index + 1) % 4;
            side2_end_i = (closest_point_index + 2) % 4;
        }

        geo::Point side1_start = points[side1_start_i];
        geo::Point side1_end = points[side1_end_i];
        geo::Offset side1_offset = side1_end - side1_start;

        geo::Point side2_start = points[side2_start_i];
        geo::Point side2_end = points[side2_end_i];
        geo::Offset side2_offset = side2_end - side2_start;

        for (int i = 0; i <= num_subdivisions; i++)
        {
            double t = static_cast<double>(i) / num_subdivisions;
            geo::Point p_side1 = side1_start + side1_offset * t;
            geo::Point p_side2 = side2_start + side2_offset * t;

            _points_side1.push_back(p_side1);
            _points_side2.push_back(p_side2);
        }

        bool start_from_side1 = true;
        for (int i = 0; i <= num_subdivisions; i++)
        {
            if (start_from_side1)
            {
                _path.push_back(_points_side1[i]);
                _path.push_back(_points_side2[i]);
            }
            else
            {
                _path.push_back(_points_side2[i]);
                _path.push_back(_points_side1[i]);
            }
            start_from_side1 = !start_from_side1;
        }
    }

    const std::vector<geo::Point> &path() const { return _path; }

private:
    const geo::Point _home;
    std::vector<geo::Point> _points_side1;
    std::vector<geo::Point> _points_side2;
    std::vector<geo::Point> _path;
};
