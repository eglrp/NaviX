#include "serial/serial.h"

void SerialHelper::set_odom(double x, double y, double th)
{
    if (fabs(th) >= 6.28)
    {
        std::cerr<<"(set_odom(x,y,th) th is represent as rad)"<<std::endl;
    }
    gx_ = x;
    gy_ = y;
    gth_ = th;
}

void SerialHelper::base_vel_callback(const char* topic, const VelocityMsg3& msg)
{
    base_vel_msg.vx = msg.vx;
    base_vel_msg.vy = msg.vy;
    base_vel_msg.wth = msg.wth;
}

void SerialHelper::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    const float alpha = 0.5;
    romote_vel_msg.vx = msg->axes[1] * alpha;
    romote_vel_msg.vy = msg->axes[0] * alpha;
    romote_vel_msg.wth = msg->axes[3] * alpha * 2;
    if(msg->buttons[Y] == 1)
        is_romote_ = true;
    else
        is_romote_ = false;
}

void SerialHelper::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_msg.vx = msg->linear.x;
    cmd_vel_msg.vy = msg->linear.y;
    cmd_vel_msg.wth = msg->angular.z;
    is_romote_ = false;/**/
}

int SerialHelper::run(void)
{
    ros::Rate rate(rate_);
    tf::TransformBroadcaster broadcaster;

    ros::Subscriber sub_joy = node.subscribe("joy", 10, &SerialHelper::joy_callback, this);
    ros::Subscriber sub_cmd_vel = node.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SerialHelper::cmd_vel_callback, this);
    ros::Publisher  odom_pub = node.advertise<nav_msgs::Odometry>("/odom", 1);

    serial.subscribe<VelocityMsg3>(0x20,"[Global Velocity]", &SerialHelper::base_vel_callback, this);

    geometry_msgs::Quaternion odom_quat;
    nav_msgs::Odometry odom;
    ros::Time current_time,last_time;

    while(ros::ok())
    {
        boost::this_thread::interruption_point();
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (base_vel_msg.vx * cos(gth_) - base_vel_msg.vy * sin(gth_)) * dt;
        double delta_y = (base_vel_msg.vx * sin(gth_) + base_vel_msg.vy * cos(gth_)) * dt;
        double delta_th = base_vel_msg.wth * dt;

        gx_  += delta_x;
        gy_  += delta_y;
        gth_ += delta_th;

        odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,gth_);

        // update transform
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = gx_;
        odom_trans.transform.translation.y = gy_;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        broadcaster.sendTransform(odom_trans);

        // filling the odometry
        odom.header.stamp =  ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        // position
        odom.pose.pose.position.x = gx_;
        odom.pose.pose.position.y = gy_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = base_vel_msg.vx;
        odom.twist.twist.linear.y = base_vel_msg.vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = base_vel_msg.wth;
        odom_pub.publish(odom);
        last_time = current_time;

        if(is_romote_ == true)
            serial.publish(0x10, &romote_vel_msg);
        else
            serial.publish(0x10, &cmd_vel_msg);

        ros::spinOnce();
        rate.sleep();
    }
}


