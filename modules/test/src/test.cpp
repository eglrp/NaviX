#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
const double limit = 0.3;

double rad2ang(double rad)
{
    return rad*360.0/3.14;
}

double relu_acc(const double acc, const double limit, double x)
{
/*
 * acc : same like k in liner fuction y=kx+b
 * limit: y=kx+b which y = 0
 */
    double y = 0;
    if(x < limit)
    {
        y = 0;
    }
    else
    {
        double b = limit * acc;
        y = acc * x - b;
        if(y <= 0) y = 0;
        if(y >= 1) y = 1;
    }
    return y;
}

void laserCB(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    double output = 0;
    std::vector<float>::const_iterator obstacle = std::min_element(laser->ranges.begin(), laser->ranges.end());
    int index = std::distance(laser->ranges.begin(), obstacle);
    double angle = laser->angle_min + index * laser->angle_increment;
    output = relu_acc(5.0, 0.3, *obstacle);
    printf("cost:%6.2f angle:%6.2f range:%6.2f\r\n", output, rad2ang(angle), laser->ranges[index]);
    fflush(stdout);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 20, laserCB);
  ros::spin();
  return(0);
}
