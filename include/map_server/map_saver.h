#ifndef MAP_SERVER_MAP_SAVER_H
#define MAP_SERVER_MAP_SAVER_H

#include <cstdio>
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

class MapGenerator
{
  public:
    MapGenerator(const std::string& mapname, ros::NodeHandle& n) : mapname_(mapname), saved_map_(false), n_(n)
    {}

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);

    int run(void);

private:
    ros::NodeHandle n_;
    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
};

#endif
