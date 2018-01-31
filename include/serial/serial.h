#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "serial/node_handle.h"
#include "hardwareSerial.h"

#include "msgs/VelocityMsg3.hpp"
#include "msgs/OdometryMsg.hpp"


typedef serial::NodeHandle<HardwareSerial> NodeHandleSerial;


class SerialHelper
{
private:
    ros::NodeHandle node;
    NodeHandleSerial serial;

    VelocityMsg3 base_vel_msg;
    VelocityMsg3 romote_vel_msg;
    VelocityMsg3 cmd_vel_msg;

    bool is_romote_;
    double rate_;
    double gx_, gy_, gth_;

    enum JOY_BUTTON
    {
        A = 0,
        B = 1,
        X = 3,
        Y = 4,
        LEFT_BUTTON = 6,
        RIGHT_BUTTON = 7,
        BACK = 10,
        START = 11,
    };
    enum JOY_AXES
    {
        LEFT_STICK_LR = 0,
        LEFT_STICK_UD = 1,
        RIGHT_STICK_LR = 2,
        RIGHT_STICK_UD = 3,
        LEFT_TRIGGER = 5,  /* start with 1.0 */
        RIGHT_TRIGGER = 4, /* start with 1.0 */
        DIRECTIONAL_PAD_LR = 6, /* only +1 and -1 */
        DIRECTIONAL_PAD_UP = 7,    /* only +1 and -1 */
    };

public:
    SerialHelper(ros::NodeHandle& n):rate_(50),is_romote_(false), node(n)
    {}

    void set_odom(double x, double y, double th);

    void base_vel_callback(const char* topic, const VelocityMsg3& msg);

    void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

    int run(void);
};

#endif
