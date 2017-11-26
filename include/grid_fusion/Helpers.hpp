#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <cmath>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include "Point.hpp"
#include "Constants.hpp"

namespace robot {

    double rad2deg(double angle);
    double angleDiff(double a1, double a2);
    double getYaw(const geometry_msgs::Quaternion& msg_q);
    int argMin(double phi, double* thetas);
    void updateOccupancyGrid(double* grid, std::vector<float> readings, Point position, double heading, double beamWidth, double beamMax, float zMax);
    double* getSensorHeadings(double heading);
    double getInverseSensorModel(Point mp, Point cp, float zMax, std::vector<float> z, double* thetas, double beamWidth);
    bool inPerceptualField(Point other, Point cur, double heading, double beamMax, float zMax);
    void runOccupancyGridMapping(const sensor_msgs::LaserScan::ConstPtr& msg, Point position, double heading, double* grid);

}

#endif // HELPERS_HPP
