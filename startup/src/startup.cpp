#include <iostream>

#include "thread_group.hpp"

void mySigintHandler(int sig)
{
    std::cout<<"------------------ Shutting Down ---------------------"<<std::endl;
    ros::shutdown();
    std::cout<<"------------------------------------------------------"<<std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navix", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    ros::NodeHandle n;

    TaskGroup test;
    test.run({TaskGroup::SECOND,
              TaskGroup::GMAPPING,
              TaskGroup::HOKUYO,
              TaskGroup::MAPSAVER,
              TaskGroup::TRANSFORM,
              TaskGroup::JOYSTICK});
    signal(SIGINT, mySigintHandler);
    while(ros::ok())
    {
        sleep(10);
        test.run({TaskGroup::MAPSAVER});
    }
}
