/**
 * @author nconlon
 */

#include "grid_fusion/Robot.hpp"

namespace robot
{
    Robot::Robot(ros::NodeHandle nhPtr,
                 const char* laserScanSubTopic,
                 const char* odometrySubTopic,
                 const char* twistPubTopic,
                 const char* gridPubTopic,
                 const char* arrayPubTopic)
    {
        laserScanSub = nhPtr.subscribe<sensor_msgs::LaserScan>(laserScanSubTopic, 1, &Robot::laserScanCallback, this);
        odometrySub = nhPtr.subscribe<nav_msgs::Odometry>(odometrySubTopic, 1, &Robot::odometryCallback, this);
        twistPub = nhPtr.advertise<geometry_msgs::Twist>(twistPubTopic, 1);
        mapPub = nhPtr.advertise<nav_msgs::OccupancyGrid>(gridPubTopic, 1);
        arrayPub = nhPtr.advertise<std_msgs::Float32MultiArray>(arrayPubTopic, 1);

        xPos = yPos = 0;
        moving = false;
        initGrid();
    }


    bool collision()
    {
//        double x = 0;
//        double y = 0;
//        for(int i=0; i<10; i++)
//        {
//            x+=prevXPos[i];
//            y+=prevYPos[i];
//        }
//        x/=10;
//        y/=10;
//        if(fabs(x-xPos)< 0.005 && fabs(y-yPos) < 0.005)
//        {
//            return true;
//            std::cout << "collision detected" << std::endl;
//        }
        return false;

    }

    void Robot::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if(moving)
        {
            m_vel.linear.x = 5.0;
            m_vel.angular.z = 0.0;

            if(msg->ranges[2] < 2)
            {
                m_vel.linear.x = 0.0;
                m_vel.angular.z = 10;
            }
            else if(collision())
            {
                m_vel.linear.x = -1;
                m_vel.angular.z = 10;
            }
            else if(msg->ranges[1] < msg->ranges[3])
            {
                m_vel.angular.z = 2;
            }
            else
            {
                m_vel.angular.z = -2;
            }

            runOccupancyGridMapping(msg, Point(xPos+scale/2.0, yPos+scale/2.0), heading, grid);

            for(int i = 0; i < maxX*maxY; i++)
            {
                m_grid.data.at(i) = grid[i];

                double prob = 1-(1.0/(1+exp(grid[i])));


                if(prob < 0.2)
                    m_map.data.at(i) = 0; // empty
                else if(prob > 0.65) //TODO  tweak me
                    m_map.data.at(i) = 100; // occupied
                else
                    m_map.data.at(i) = -1; // unknown
            }

            mapPub.publish(m_map);
            arrayPub.publish(m_grid);
            twistPub.publish(m_vel);
        }
    }


    void Robot::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        moving = true;
        xPos = msg->pose.pose.position.x+(maxX/scale)/2;
        yPos = msg->pose.pose.position.y+(maxY/scale)/2;
        heading = getYaw(msg->pose.pose.orientation);
    }

    void Robot::initGrid()
    {
        //Create a header, populate the fields.
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        header.frame_id = "/map";

        //Create the map meta data
        nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
        metaD.map_load_time = ros::Time::now();
        metaD.resolution = resolution; //each pixel will represent .05 meters
        metaD.width = maxX; //2400 pixels long , AKA 120 meters
        metaD.height = maxY; //600 pixels tall, AKA 30 meter
        //metaD.origin will just init to 0, no need to change

        m_map = nav_msgs::OccupancyGrid();
        m_map.header = header;
        m_map.info = metaD;

        for(int i = 0; i < maxX*maxY; i++)
        {
            grid[i] = 0.0; // likelihood map
            m_map.data.push_back(-1);  // occupancy grid to publish
            m_grid.data.push_back(0.0); // likelihood map to publish
        }
    }

    Robot::~Robot()
    {
        ;
    }
}

