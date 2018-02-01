#include <iostream>
#include "thread_group.hpp"

ModuleGroup test;
ros::Publisher pub;

void mySigintHandler(int sig)
{
    std::cout<<"------------------ Shutting Down ---------------------"<<std::endl;
    ros::shutdown();
    std::cout<<"------------------------------------------------------"<<std::endl;
}

void joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    ros::NodeHandle n;
    geometry_msgs::Twist vel;
    if(msg->buttons[JOY::Y] == JOY::PRESSED)
    {
        vel.linear.x  = msg->axes[JOY::LEFT_STICK_UD]*0.1;
        vel.angular.z = msg->axes[JOY::LEFT_STICK_LR]*0.1;
    }
    if(msg->buttons[JOY::X] == JOY::PRESSED)
    {
        test.run({ModuleGroup::MAPSAVER});
    }
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub.publish(vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navix", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    ros::NodeHandle n;
    ros::Subscriber joy_sub_ = n.subscribe("joy", 1, &joyCallback);

    test.run({ModuleGroup::SECOND,
              ModuleGroup::GMAPPING,
              ModuleGroup::HOKUYO,
              ModuleGroup::MAPSAVER,
              ModuleGroup::TRANSFORM,
              ModuleGroup::JOYSTICK});
    signal(SIGINT, mySigintHandler);
    while(ros::ok())
    {
        ros::spinOnce();
    }
}
