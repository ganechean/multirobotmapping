/**
 * @author nconlon
 */

#include "grid_fusion/Fuser.hpp"

namespace robot
{
    Fuser::Fuser(ros::NodeHandle nhPtr,
                 const char *gridPubTopic,
                 const char *floatArraySubTopic,
                 const char *intArraySubTopic)
    {
        arraySub = nhPtr.subscribe<std_msgs::Float32MultiArray>(floatArraySubTopic, 5, &Fuser::arrayCallback, this);
        intArraySub = nhPtr.subscribe<std_msgs::Int8MultiArray>(intArraySubTopic, 5, &Fuser::intArrayCallback, this);

        gridPub = nhPtr.advertise<nav_msgs::OccupancyGrid>(gridPubTopic, 1);

        init();
    }

    void Fuser::arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        fuseFloat(msg);
        gridPub.publish(m_map);
    }

    void Fuser::intArrayCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        fuseInt(msg);
        gridPub.publish(m_map);
    }


    void Fuser::fuseInt(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        switch(FUSION)
        {
        case IMSF:
            runInt(msg);
            break;
        }
    }

    void Fuser::fuseFloat(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        switch(FUSION)
        {
        case IOP:
            runIOP(msg);
            break;
         case MAX :
            runMax(msg);
            break;
        }
    }

    void Fuser::runMax(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           float probNew = 1-(1.0/(1+exp(msg->data.at(i))));
           float probOld = grid.at(i);

           float prob = fuseUsingMaximization(probNew, probOld);

           grid.at(i) = prob;

           if(prob < EMPTY_THRESHOLD_FLOAT)
               m_map.data.at(i) = 0; // empty
           else if(prob > OCCUPIED_THRESHOLD_FLOAT)
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }
    }

    void Fuser::runIOP(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           float probNew = 1-(1.0/(1+exp(msg->data.at(i))));
           float probOld = grid.at(i);

           float prob = fuseUsingIndependentOpinionPool(probNew, probOld);

           grid.at(i) = prob;

           if(prob < EMPTY_THRESHOLD_FLOAT)
               m_map.data.at(i) = 0; // empty
           else if(prob > OCCUPIED_THRESHOLD_FLOAT)
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }
    }

    void Fuser::runInt(const std_msgs::Int8MultiArray::ConstPtr& msg)
    {
        for(int i=0; i<msg->data.size(); i++)
        {
           uint8_t indexNew = 1;
           uint8_t indexOld = 1;

           uint8_t prob = fuseUsingIntegerArithmetic(indexNew, indexOld);

           int_grid.at(i) = prob;

           if(prob < EMPTY_THRESHOLD_FLOAT)
               m_map.data.at(i) = 0; // empty
           else if(prob > OCCUPIED_THRESHOLD_FLOAT)
               m_map.data.at(i) = 100; // occupied
           else
               m_map.data.at(i) = -1; // unknown
        }
    }

    float Fuser::fuseUsingMaximization(float probA, float probB)
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

    float Fuser::fuseUsingIndependentOpinionPool(float probA, float probB)
    {
        float result =  (probA*probB)/(probA*probB + (1-probA)*(1-probB));
        return result;
    }

    uint8_t Fuser::fuseUsingIntegerArithmetic(uint8_t a, uint8_t b)
    {
        return 0;
    }


    void Fuser::init()
    {
        //Create a header, populate the fields.
        std_msgs::Header header = std_msgs::Header();
        header.stamp = ros::Time::now();
        header.frame_id = "/map";

        //Create the map meta data
        nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
        metaD.map_load_time = ros::Time::now();
        metaD.resolution = RESOLUTION;
        metaD.width = MAX_X;
        metaD.height = MAX_Y;

        m_map = nav_msgs::OccupancyGrid();
        m_map.header = header;
        m_map.info = metaD;

        for(int i = 0; i < MAX_X*MAX_Y; i++)
        {
            grid.push_back(0.5);
            int_grid.push_back(0);
            m_map.data.push_back(-1);
        }
    }
}
