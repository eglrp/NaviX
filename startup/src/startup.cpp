#include <iostream>
#include "thread_group.hpp"

ModuleGroup test;

void mySigintHandler(int sig)
{
    std::cout<<"------------------ Shutting Down ---------------------"<<std::endl;
    ros::shutdown();
    std::cout<<"------------------------------------------------------"<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navix", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    test.run({ModuleGroup::GMAPPING,
              ModuleGroup::HOKUYO,
              ModuleGroup::TRANSFORM,
              ModuleGroup::JOYSTICK,
              ModuleGroup::TASK});
    signal(SIGINT, mySigintHandler);
    ros::spin();
    return 0;
}
