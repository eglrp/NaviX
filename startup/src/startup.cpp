#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>

#include "joy/joy.h"
#include "gmapping/openslam/slam_gmapping.h"
#include "urg_node/urg_node_driver.h"
#include "serial/serial.h"
#include "thread_assist/thread_assist.hpp"

class testThread
{
public:
    int driver_joy_thread()
    {
        Joystick joy;
        joy.run();
        return 0;
    }

    int driver_serial_thread()
    {
        ros::NodeHandle n;
        SerialHelper serial(n);
        serial.run();
    }
};

testThread test;
ThreadAssist ta;

namespace Thread_Driver
{
    int driver_joy_thread()
    {
        Joystick joy;
        joy.run();
        return 0;
    }

    int driver_laser_thread()
    {

        urg_node::UrgNode node;
        node.run();
        printf("[LASER] Name: %s, ID: %s\r\n",node.productName().c_str(), node.deviceID().c_str());
        while(ros::ok())
        {
            INTERRUPT_LOOP();
            sleep(1);
        }
        return 0;
    }

    int slam_gmapping()
    {
        SlamGMapping gm;
        gm.startLiveSlam();
        while(ros::ok())
        {
            INTERRUPT_LOOP();
            sleep(1);
        }
        return 0;
    }

    int second_thread()
    {
        int i=0;
        while(ros::ok())
        {
            INTERRUPT_LOOP();
            printf("second : %d\r\n",i);
            fflush(stdout);
            i++;
            sleep(1);
        }
        return 0;
    }

    int move_base_thread()
    {
        int i=0;
        while(ros::ok())
        {
            INTERRUPT_LOOP();
        }
        return 0;
    }

//    void poseCallback(const turtlesim::PoseConstPtr& msg)
//    {
//      static tf::TransformBroadcaster br;
//      tf::Transform transform;
//      tf::Quaternion q;
//      transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
//      q.setRPY(0, 0, msg->theta);
//      transform.setRotation(q);
//      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
//    }

}


void mySigintHandler(int sig)
{
    // Do some custom action.
    // For e\ample, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    std::cout<<"------------------ Shutting Down ---------------------"<<std::endl;
    ta.shutdown_all();
    ta.show_tables();
    ros::shutdown();
    std::cout<<"------------------------------------------------------"<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navix", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);

    ta.add_tables("SECOND",   &Thread_Driver::second_thread);
    ta.add_tables("JOYSTICK", &testThread::driver_joy_thread, test);
    ta.add_tables("SERIAL",   &testThread::driver_serial_thread, test);
    ta.add_tables("HOKUYO",   &Thread_Driver::driver_laser_thread);
    ta.add_tables("GMAPPING", &Thread_Driver::slam_gmapping);// at last, because join need time.

    while(n.ok())
    {
        ros::spinOnce();
    }
    ta.shutdown_all();
}
