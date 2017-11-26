/**
 * @author nconlon
 */

#include "grid_fusion/Fuser.hpp"

namespace robot
{
    Fuser::Fuser(ros::NodeHandle nhPtr, const char *gridSubTopic, const char *gridPubTopic, const char *ArraySubTopic)
    {
        arraySub = nhPtr.subscribe<std_msgs::Float32MultiArray>(ArraySubTopic, 5, &Fuser::arrayCallback, this);
        gridPub = nhPtr.advertise<nav_msgs::OccupancyGrid>(gridPubTopic, 1);

        init();
    }

    void Fuser::arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        fuse(msg);
        gridPub.publish(m_map);
    }

    void Fuser::fuse(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           double probNew = 1-(1.0/(1+exp(msg->data.at(i))));
           double probOld = grid.at(i);

           //double prob = fuseIOP(probNew, probOld);
           double prob = fuseMax(probNew, probOld);

           grid.at(i) = prob;

           if(prob < emptyThreshold)
               m_map.data.at(i) = 0; // empty
           else if(prob > occupiedThreshold)
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }

    }

    double Fuser::fuseMax(double probA, double probB)
    {
        if(probA < 0.5 || probB < 0.5)
        {
            return std::min(probA, probB);
        }
        else if(probA > 0.5 || probB > 0.5)
        {
            return std::max(probA, probB);
        }
        else
        {
            return 0.5;
        }
    }

    double Fuser::fuseIOP(double probA, double probB)
    {
        double result =  (probA*probB)/(probA*probB + (1-probA)*(1-probB));
        return result;
    }

    void Fuser::init()
    {
        // set the thresholds
        emptyThreshold = 1-(1.0/(1+exp(-20)));
        occupiedThreshold = 1-(1.0/(1+exp(20)));

        //Create a header, populate the fields.
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        header.frame_id = "/map";

        //Create the map meta data
        nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
        metaD.map_load_time = ros::Time::now();
        metaD.resolution = resolution;
        metaD.width = maxX;
        metaD.height = maxY;

        m_map = nav_msgs::OccupancyGrid();
        m_map.header = header;
        m_map.info = metaD;

        for(int i = 0; i < maxX*maxY; i++)
        {
            grid.push_back(0.5);
            m_map.data.push_back(-1);
        }
    }
}
