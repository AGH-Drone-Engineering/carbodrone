#pragma once

#include <GeographicLib/Geodesic.hpp>

namespace geo
{

static void solve_inverse(double lat0, double lon0, double lat1, double lon1, double *x, double *y)
{
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    double s12, azi1, azi2;
    geod.Inverse(lat0, lon0, lat1, lon1, s12, azi1, azi2);
    double angle = (-azi1 + 90) * M_PI / 180;
    *x = s12 * cos(angle);
    *y = s12 * sin(angle);
}

static void solve_direct(double lat0, double lon0, double dx, double dy, double *lat1, double *lon1)
{
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    double azi1, azi2, s12;
    azi1 = atan2(dx, dy) * 180 / M_PI;
    s12 = sqrt(dx * dx + dy * dy);
    geod.Direct(lat0, lon0, azi1, s12, *lat1, *lon1, azi2);
}

class Offset
{
public:
    Offset(double dx, double dy)
        : _dx(dx)
        , _dy(dy)
    {
    }

    Offset() = delete;
    Offset(const Offset &other) = default;
    Offset &operator=(const Offset &other) = default;

    Offset operator*(double scalar) const
    {
        return Offset(_dx * scalar, _dy * scalar);
    }

    double dx() const { return _dx; }
    double dy() const { return _dy; }

private:
    double _dx;
    double _dy;
};

class Point
{
public:
    Point(double lat, double lon)
        : _lat(lat)
        , _lon(lon)
    {
    }

    Point() = delete;
    Point(const Point &other) = default;
    Point &operator=(const Point &other) = default;

    double lat() const { return _lat; }
    double lon() const { return _lon; }

    Point operator+(const Offset &offset) const
    {
        double lat, lon;
        double dx = offset.dx();
        double dy = offset.dy();
        solve_direct(_lat, _lon, dx, dy, &lat, &lon);
        return Point(lat, lon);
    }

    double distance(const Point &other) const
    {
        auto offset = other - *this;
        return sqrt(offset.dx() * offset.dx() + offset.dy() * offset.dy());
    }

    Offset operator-(const Point &other) const
    {
        double dx, dy;
        solve_inverse(other._lat, other._lon, _lat, _lon, &dx, &dy);
        return Offset(dx, dy);
    }

private:
    double _lat;
    double _lon;
};

}; // namespace geo
