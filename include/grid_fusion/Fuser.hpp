#ifndef FUSER_HPP
#define FUSER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream> //I/O
#include <string>
#include <memory>   //shared pointers
#include <algorithm>

#include "Constants.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace robot {

    class Fuser
    {
    public:
        Fuser(ros::NodeHandle nhPtr,
              const char* gridSubTopic,
              const char* gridPubTopic,
              const char* ArraySubTopic);
        void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

        ~Fuser();

    private:
        std::vector<double> grid;
        ros::Subscriber gridSub;
        ros::Subscriber arraySub;
        ros::Publisher gridPub;
        nav_msgs::OccupancyGrid m_map;
        void fuse(const std_msgs::Float32MultiArray::ConstPtr& msg);
        double fuseMax(double a, double b);
        double fuseIOP(double a, double b);
    };

}
#endif // FUSER_HPP
