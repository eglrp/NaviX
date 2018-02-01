#ifndef __THREAD_GROUP__
#define __THREAD_GROUP__

#include <initializer_list>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <boost/shared_ptr.hpp>

#include "joy/joy.h"
#include "gmapping/openslam/slam_gmapping.h"
#include "urg_node/urg_node_driver.h"
#include "serial/serial.h"
#include "map_server/map_saver.h"

#include "thread_assist/thread_assist.hpp"

class TaskGroup
{
private:
    boost::shared_ptr<ThreadAssist> thread_assist;

    typedef struct
    {
        uint32_t id;
        int (TaskGroup::*ptr)(void);
        const char* info;
    }Groups;
    vector<Groups> groups_;

public:
    enum MODULES
    {
        SECOND = 0,
        TRANSFORM,
        JOYSTICK,
        SERIAL,
        MAPSAVER,
        HOKUYO,
        GMAPPING
    };

    TaskGroup()
    {
        thread_assist.reset(new ThreadAssist());
        groups_.push_back({(int)SECOND,    &TaskGroup::second_thread,        "SECOND"});
        groups_.push_back({(int)TRANSFORM, &TaskGroup::ros_transform_thread, "TRANSFORM"});
        groups_.push_back({(int)JOYSTICK,  &TaskGroup::driver_joy_thread,    "JOYSTICK"});
        groups_.push_back({(int)SERIAL,    &TaskGroup::driver_serial_thread, "SERIAL"});
        groups_.push_back({(int)MAPSAVER,  &TaskGroup::map_saver_thread,     "MAPSAVER"});
        groups_.push_back({(int)HOKUYO,    &TaskGroup::driver_laser_thread,  "HOKUYO"});
        groups_.push_back({(int)GMAPPING,  &TaskGroup::slam_gmapping,        "GMAPPING"});
    }

    void run(initializer_list<MODULES> param)
    {
        for(auto p = param.begin(); p != param.end(); p++)
        {
            for(auto g = groups_.begin(); g!=groups_.end();g++)
            {
                if ((int)*p == g->id)
                {
                    thread_assist->add_tables(g->info, g->ptr, *this);
                }
            }
        }
        thread_assist->fresh_tables();
    }

    void shutdown()
    {
        thread_assist->shutdown_all();
    }

private:
    int ros_transform_thread()
    {
        ros::NodeHandle n;
        ros::Rate rate(50);
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        while(ros::ok())
        {
            INTERRUPT_LOOP();
            transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));

            transform.setOrigin(tf::Vector3(0.05, 0.0, 0.0));
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "laser"));
            rate.sleep();
        }
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
    }

    int map_saver_thread()
    {
        ros::NodeHandle n;
        MapGenerator saver("map", n);
        saver.run();
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
        return 0;
    }

    int driver_joy_thread()
    {
        Joystick joy;
        joy.run();
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
        return 0;
    }

    int driver_serial_thread()
    {
        ros::NodeHandle n;
        SerialHelper serial(n);
        serial.run();
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
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
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
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
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
        return 0;
    }

    int second_thread()
    {
        int i=0;
        while(ros::ok())
        {
            INTERRUPT_LOOP();
            thread_assist->show_tables();
            sleep(1);
            fflush(stdout);
        }
        thread_assist->setStatus(boost::lexical_cast<std::string>(boost::this_thread::get_id()), ThreadAssist::Stop);
        return 0;
    }

    int move_base_thread()
    {
        int i=0;
        while(ros::ok())
        {
            INTERRUPT_LOOP();
            sleep(1);
        }

        return 0;
    }
};

#endif
