
#include "grid_fusion/Helpers.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream> //I/O

namespace robot
{


    void runOccupancyGridMapping(const sensor_msgs::LaserScan::ConstPtr& msg, Point position, double heading, double* grid)
    {
        double beamMax = fmod(rad2deg(msg->angle_max)+360, 360);
        double beamWidth = beamMax/5.0;
        updateOccupancyGrid(grid, msg->ranges, position, heading, beamWidth, beamMax, msg->range_max);
    }

    void updateOccupancyGrid(double* grid,
                             std::vector<float> readings,
                             Point position,
                             double heading,
                             double beamWidth,
                             double beamMax,
                             float zMax)
    {
        double* sensorHeadings = getSensorHeadings(heading);

        for(int y = 0; y < maxY; y++)
        {
            for(int x = 0; x < maxX; x++)
            {
                Point mp = Point(x/scale+scale/2.0,y/scale+scale/2.0);

                if(inPerceptualField(mp, position, sensorHeadings[2], beamMax, zMax))
                {
                    double log = getInverseSensorModel(mp, position, zMax, readings, sensorHeadings, beamWidth) - 0.0;
                    grid[y*maxX+x] = grid[y*maxX+x]+log;
                    if(grid[y*maxX+x] > 100)
                    {
                       grid[y*maxX+x]  = 100;
                    }
                    if(grid[y*maxX+x] < -100)
                    {
                        grid[y*maxX+x] = -100;
                    }

                }
            }
        }
    }

    double getInverseSensorModel(Point mp,
                                 Point cp,
                                 float zMax,
                                 std::vector<float> z,
                                 double* thetas,
                                 double beamWidth)
    {
        float alpha = 2.0/scale;
        double beta = beamWidth;
        double r = mp.getDistance(cp);
        double phi = fmod(rad2deg(atan2(mp.getY()-cp.getY(), mp.getX()-cp.getX()))+360,360);
        int k = argMin(phi, thetas);

        if(r > std::min(zMax, z.at(k))|| angleDiff(phi, thetas[k]) > beta/2)
        {
            return 0.0; //l_o
        }
        else if(z.at(k) < zMax && fabs(r-z.at(k)) < alpha)
        {
            return 2.0; //l_occ
        }
        else
        {
            return -2.0; // l_free
        }
    }


    bool inPerceptualField(Point other, Point cur, double heading, double beamMax, float zMax)
    {
        double dist = other.getDistance(cur);
        double theta = atan2(other.getY()-cur.getY(), other.getX()-cur.getX());
        theta = fmod((rad2deg(theta)+360.0),360.0);
        double diff = angleDiff(heading, theta);
        if(dist <= zMax && diff <= beamMax)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    double rad2deg(double rad)
    {
        return rad* (180/M_PI);
    }

    double angleDiff(double a1, double a2)
    {
        a1 = fmod(a1+360.0,360.0);
        a2 = fmod(a2+360.0,360.0);
        return 180-abs(abs(a1-a2)-180.0);
    }

    double getYaw(const geometry_msgs::Quaternion& msg_q)
    {
       return tf::getYaw(msg_q);
    }


    int argMin(double phi, double* thetas)
    {
        double k = 360.0;
        int i = 0;
        int size = sizeof(thetas)/sizeof(thetas[0]);
        for(int j = 0; j<5; j++)
        {
            double arg = abs(phi-thetas[j]);
            if(arg < k)
            {
             k = arg;
             i = j;
            }
        }
        return i;
    }



    double* getSensorHeadings(double heading)
    {
        double* array = new double[5];
        array[0] = fmod(rad2deg(heading-M_PI/2.0)+360,360);
        array[1] = fmod(rad2deg(heading-M_PI/4.0)+360,360);
        array[2] = fmod(rad2deg(heading)+360,360);
        array[3] = fmod(rad2deg(heading+M_PI/4.0)+360,360);
        array[4] = fmod(rad2deg(heading+M_PI/2.0)+360,360);

        return array;
    }


}






