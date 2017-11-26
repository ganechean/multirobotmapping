/**
 * @author nconlon
 */


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

        /**
         * Callback for the Float 32 array message
         *
         * @brief arrayCallback
         * @param msg
         */
        void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

        ~Fuser();

    private:
        // pubs and subs
        ros::Subscriber arraySub;
        ros::Publisher gridPub;

        // the fused occupancy grid to publish
        nav_msgs::OccupancyGrid m_map;

        // local fused likelihood map
        std::vector<double> grid;

        // thresholds for the occupancy grid
        double emptyThreshold;
        double occupiedThreshold;

        // fuse msg with grid
        void fuse(const std_msgs::Float32MultiArray::ConstPtr& msg);

        // fuse based on simple max
        double fuseMax(double a, double b);

        // fuse based on the IOP method
        double fuseIOP(double a, double b);

        // initialize the node
        void init();
    };

}
#endif // FUSER_HPP
