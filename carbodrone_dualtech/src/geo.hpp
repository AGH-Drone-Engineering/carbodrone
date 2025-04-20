#pragma once

#include <GeographicLib/Geodesic.hpp>

static void geo_solve_inverse(double lat0, double lon0, double lat1, double lon1, double *x, double *y)
{
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    double s12, azi1, azi2;
    geod.Inverse(lat0, lon0, lat1, lon1, s12, azi1, azi2);
    double angle = (-azi1 + 90) * M_PI / 180;
    *x = s12 * cos(angle);
    *y = s12 * sin(angle);
}

static void geo_solve_direct(double lat0, double lon0, double dx, double dy, double *lat1, double *lon1)
{
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    double azi1, azi2, s12;
    azi1 = atan2(dx, dy) * 180 / M_PI;
    s12 = sqrt(dx * dx + dy * dy);
    geod.Direct(lat0, lon0, azi1, s12, *lat1, *lon1, azi2);
}
