#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <cmath>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <simple_local_planner/SimpleLocalPlannerConfig.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "pure_pursuit.h"
#include "pid.h"

namespace simple_local_planner {

class SimpleLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
    void initialize(
            std::string name,
            tf::TransformListener* tf,
            costmap_2d::Costmap2DROS* costmap_ros)
    {
        tf_ = tf;

        current_waypoint_ = 0;
        is_goal_reach_ = false;
        goal_reached_time_ = ros::Time::now();

        ros::NodeHandle node_private("~/SimpleLocalPlanner");
        marker_pub1 = node_private.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
        marker_pub2 = node_private.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
        marker_pub3 = node_private.advertise<visualization_msgs::Marker>("visualization_marker3", 10);
        marker_pub4 = node_private.advertise<visualization_msgs::Marker>("visualization_marker4", 10);

        node_private.param("tolerance_trans", tolerance_trans_, 0.01);
        node_private.param("tolerance_rot", tolerance_rot_, 0.01);
        node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

        node_private.param("holonomic", holonomic_, true);

        node_private.param("samples", samples_, 1);
        node_private.param("steps", steps_, 1);

        node_private.param("max_vel_lin", max_vel_lin_, 0.4);
        node_private.param("min_vel_lin", min_vel_lin_, 0.1);

        node_private.param("max_vel_th", max_vel_th_, 2.0);
        node_private.param("min_vel_th", min_vel_th_, 0.1);

        node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.1);
        node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.3);

        node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
        node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

        node_private.param("robot_base_frame", p_robot_base_frame_, std::string("base_link"));
        node_private.param("global_frame", p_global_frame_, std::string("map"));

        dsrv_ = new dynamic_reconfigure::Server<SimpleLocalPlannerConfig>(node_private);
        dynamic_reconfigure::Server<SimpleLocalPlannerConfig>::CallbackType cb = boost::bind(&SimpleLocalPlanner::reconfigure, this, _1, _2);
        dsrv_->setCallback(cb);

        pid_vx_.init(2, 0.0, 0, 1);
        pid_vx_.set_max_min_output(max_vel_lin_, -max_vel_lin_);

        pid_vy_.init(3, 0.1, 0, 1);
        pid_vy_.set_max_min_output(max_vel_lin_, -max_vel_lin_);

        pid_wz_.init(2, 0.1, 0, 1);
        pid_wz_.set_max_min_output(max_vel_th_, -max_vel_th_);
    }

    void reconfigure(SimpleLocalPlannerConfig &config, uint32_t level)
     {
        boost::mutex::scoped_lock l(configuration_mutex_);

        K_x_ = config.k_liner_x;
        K_y_ = config.k_liner_y;
        K_rot_ = config.k_rot;

        tolerance_trans_ = config.tolerance_trans;
        tolerance_rot_ = config.tolerance_rot;
        tolerance_timeout_ = config.tolerance_timeout;

        holonomic_ = config.holonomic;

        samples_ = config.samples;
        steps_ = config.steps;

        max_vel_lin_ = config.max_vel_lin;
        min_vel_lin_ = config.min_vel_lin;
        max_vel_th_ = config.max_vel_th;
        min_vel_th_ = config.min_vel_th;
        min_in_place_vel_th_ = config.min_in_place_vel_th;
        in_place_trans_vel_ = config.in_place_trans_vel;

        trans_stopped_velocity_ = config.trans_stopped_velocity;
        rot_stopped_velocity_ = config.rot_stopped_velocity;
   }
    
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
        is_goal_reach_ = false;
        current_waypoint_ = 0;
        goal_reached_time_ = ros::Time::now();
        if(!transform_global_plan(*tf_, global_plan, p_global_frame_, global_plan_))
        {
          ROS_ERROR("Could not transform the global plan to the frame of the controller");
          return false;
        }
        return true;
    }
    
    bool isGoalReached()
    {
        if(is_goal_reach_ == true)
        {
            return true;
        }

        return false;
    }

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if (global_plan_.size() == 0)
          return false;

        geometry_msgs::Twist empty_twist;
        //get the current pose of the robot in the fixed frame
        tf::Stamped<tf::Pose> tf_robot_pose;
        if(!this->get_robot_pose(tf_robot_pose))
        {
          ROS_ERROR("Can't get robot pose");
          cmd_vel = empty_twist;
          return false;
        }

        double vx = last_vel.linear.x;
        ld_A = 0.01;
        ld_B = 0.01;
        ld_C = 0.05;
        ld = ld_A * vx * vx + ld_B * vx + ld_C;

        tf::Stamped<tf::Pose> tf_future_point;
        if(!this->get_future_point(tf_future_point, ld))
        {
          ROS_ERROR("Can't get future pose");
          cmd_vel = empty_twist;
          return false;
        }
        drawMarker(tf_future_point, marker_pub1);// red

        PurePursuit pure_pursuit(global_plan_);
        int closestWayPoint = pure_pursuit.getClosestWaypoint(tf_future_point);
        if(closestWayPoint >= 0 && closestWayPoint >= current_waypoint_)
        {
            current_waypoint_ = closestWayPoint;
        }

        //we want to compute a velocity command based on our current waypoint
        tf::Stamped<tf::Pose> tf_target_pose;
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], tf_target_pose);
        drawMarker(tf_target_pose, marker_pub2);//blue

        // get the difference between the two poses
        geometry_msgs::Twist diff = diff2D(tf_target_pose, tf_robot_pose);
        geometry_msgs::Twist limit_vel = limitTwist(diff);
        geometry_msgs::Twist scale_vel = scalc_forward_speed(limit_vel);
        cmd_vel = scale_vel;
        last_vel = cmd_vel;

        while(fabs(diff.linear.x) <= tolerance_trans_ &&
              fabs(diff.linear.y) <= tolerance_trans_ &&
              fabs(diff.angular.z) <= tolerance_rot_)
        {
            if(current_waypoint_ < global_plan_.size() - 1)
            {
                current_waypoint_++;
                tf::poseStampedMsgToTF(global_plan_[current_waypoint_], tf_target_pose);
                diff = diff2D(tf_target_pose, tf_robot_pose);
            }
            else
            {
                ROS_INFO("Reached goal: %d", current_waypoint_);
                is_goal_reach_ = true;
                break;
            }
        }

        //if we're not in the goal position, we need to update time
        if(!is_goal_reach_)
          goal_reached_time_ = ros::Time::now();

        //check if we've reached our goal for long enough to succeed
        if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now())
        {
          geometry_msgs::Twist empty_twist;
          cmd_vel = empty_twist;
        }
        return true;
    }

private:
    inline double rad2ang(double rad)
    {
        return rad*180.0/3.1415926;
    }

    inline double ang2rad(double ang)
    {
        return ang*3.1415926/180.0;
    }

    inline double sign(double n)
    {
        return n < 0.0 ? -1.0 : 1.0;
    }

    double headingDiff(double x, double y, double pt_x, double pt_y, double heading)
    {
        double v1_x = x - pt_x;
        double v1_y = y - pt_y;
        double v2_x = cos(heading);
        double v2_y = sin(heading);

        double perp_dot = v1_x * v2_y - v1_y * v2_x;
        double dot = v1_x * v2_x + v1_y * v2_y;

        //get the signed angle
        double vector_angle = atan2(perp_dot, dot);

        return -1.0 * vector_angle;
    }

    geometry_msgs::Twist diff2D(const tf::Pose& target_pose, const tf::Pose& robot_pose)
    {
        geometry_msgs::Twist res;
        tf::Pose diff = robot_pose.inverse() * target_pose;
        res.linear.x = diff.getOrigin().x();
        res.linear.y = diff.getOrigin().y();
        res.angular.z = tf::getYaw(diff.getRotation());

        if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
          return res;

        //in the case that we're not rotating to our goal position and we have a non-holonomic robot
        //we'll need to command a rotational velocity that will help us reach our desired heading

        //we want to compute a goal based on the heading difference between our pose and the target
        double yaw_diff = headingDiff(target_pose.getOrigin().x(), target_pose.getOrigin().y(),
                                      robot_pose.getOrigin().x(), robot_pose.getOrigin().y(),
                                      tf::getYaw(robot_pose.getRotation()));

        //we'll also check if we can move more effectively backwards
        double neg_yaw_diff = headingDiff(target_pose.getOrigin().x(), target_pose.getOrigin().y(),
                                          robot_pose.getOrigin().x(), robot_pose.getOrigin().y(),
                                          M_PI + tf::getYaw(robot_pose.getRotation()));

        //check if its faster to just back up
        if(fabs(neg_yaw_diff) < fabs(yaw_diff))
        {
          ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
          yaw_diff = neg_yaw_diff;
        }

        //compute the desired quaterion
        tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
        tf::Quaternion rot = robot_pose.getRotation() * rot_diff;
        tf::Pose new_pose = target_pose;
        new_pose.setRotation(rot);

        diff = robot_pose.inverse() * new_pose;
        res.linear.x = diff.getOrigin().x();
        res.linear.y = diff.getOrigin().y();
        res.angular.z = tf::getYaw(diff.getRotation());
    return res;
    }

    geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& diff)
    {
      geometry_msgs::Twist res = diff;
      res.linear.x = pid_vy_.calc(0, -diff.linear.x);
      res.linear.y = pid_vy_.calc(0, -diff.linear.y);

      //make sure to bound things by our velocity limits
      double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / this->max_vel_lin_;
      double lin_undershoot = this->min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
      ROS_INFO("vx:%4.2f vy:%4.2f vz:%4.2f",res.linear.x, res.linear.y, res.angular.z);
      if (lin_overshoot > 1.0)
      {
        res.linear.x /= lin_overshoot;
        res.linear.y /= lin_overshoot;
      }

      //we only want to enforce a minimum velocity if we're not rotating in place
      if(lin_undershoot > 1.0)
      {
        res.linear.x *= lin_undershoot;
        res.linear.y *= lin_undershoot;
      }

      if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
      if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);

      //we want to check for whether or not we're desired to rotate in place
      if(sqrt(diff.linear.x * diff.linear.x + diff.linear.y * diff.linear.y) < in_place_trans_vel_)
      {
        if (fabs(res.angular.z) < min_in_place_vel_th_)
            res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
        res.linear.x = res.linear.x * 0.2;
        res.linear.y = res.linear.y * 0.2;
        ROS_INFO("Angular command %f", res.angular.z);
      }

      return res;
    }

    double distance(const tf::Stamped<tf::Pose>& pt1, const tf::Stamped<tf::Pose>& pt2)
    {
        double a = pt1.getOrigin().getX()-pt2.getOrigin().getX();
        double b = pt1.getOrigin().getY()-pt2.getOrigin().getY();
        return sqrt(a*a + b*b);
    }

    geometry_msgs::Twist scalc_forward_speed(const geometry_msgs::Twist& twist)

    {
        geometry_msgs::Twist scale_speed = twist;
        double angle = rad2ang(twist.angular.z);
        // if larger than 10 degree, scale down liner x speed
        if(fabs(angle) >= 10)
        {
            scale_speed.linear.x *= 0.5;
        }
        ROS_ERROR("twist=%5.3f", angle);
        return scale_speed;
    }

    bool transform_global_plan(const tf::TransformListener& tf,
                             const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const std::string& global_frame,
                             std::vector<geometry_msgs::PoseStamped>& transformed_plan)
    {
      const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
      transformed_plan.clear();
      try
      {
        if (!global_plan.size() > 0)
        {
          ROS_ERROR("Received plan with zero length");
          return false;
        }

        tf::StampedTransform transform;
        tf.lookupTransform(global_frame, ros::Time(),
            plan_pose.header.frame_id, plan_pose.header.stamp,
            plan_pose.header.frame_id, transform);

        tf::Stamped<tf::Pose> tf_pose;
        geometry_msgs::PoseStamped newer_pose;
        //now we'll transform until points are outside of our distance threshold
        for(unsigned int i = 0; i < global_plan.size(); ++i)
        {
          const geometry_msgs::PoseStamped& pose = global_plan[i];
          poseStampedMsgToTF(pose, tf_pose);
          tf_pose.setData(transform * tf_pose);
          tf_pose.stamp_ = transform.stamp_;
          tf_pose.frame_id_ = global_frame;
          poseStampedTFToMsg(tf_pose, newer_pose);
          transformed_plan.push_back(newer_pose);
        }
      }
      catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
         false;
      }
      catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        if (global_plan.size() > 0)
          ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

        return false;
      }
      return true;
    }

    bool get_robot_pose(tf::Stamped<tf::Pose>& global_pose) const
    {
      global_pose.setIdentity();
      tf::Stamped<tf::Pose> robot_pose;
      robot_pose.setIdentity();
      robot_pose.frame_id_ = p_robot_base_frame_;
      robot_pose.stamp_ = ros::Time(0);
      ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

      //get the global pose of the robot
      try
      {
        tf_->transformPose(p_global_frame_, robot_pose, global_pose);
      }
      catch(tf::LookupException& ex) {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
      }
      catch(tf::ConnectivityException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
      }
      // check global_pose timeout
      if (current_time.toSec() - global_pose.stamp_.toSec() > 1.0) {
        ROS_WARN_THROTTLE(1.0, "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
            current_time.toSec() ,global_pose.stamp_.toSec() ,1.0);
      }
      return true;
    }

    bool get_future_point(tf::Stamped<tf::Pose>& future_pose, double look_ahead_distance) const
    {
        if (look_ahead_distance < 0)
        {
            ROS_ERROR("Look ahead distance must larger than 0.0 m, set as default 0.2.");
            look_ahead_distance = 0.2;
            return false;
        }

        future_pose.setIdentity();
        tf::Stamped<tf::Pose> robot_pose_;
        robot_pose_.setIdentity();
        robot_pose_.frame_id_ = "base_link";
        robot_pose_.stamp_ = ros::Time(0);
        robot_pose_.setOrigin(tf::Vector3(look_ahead_distance, 0, 0));
        try
        {
          tf_->transformPose("map", robot_pose_, future_pose);
        }
        catch(...)
        {
            return false;
        }
        return true;
    }

    static double normalize(double z){
        return atan2(sin(z),cos(z));
    }

    static double angle_diff(double a, double b)
    {
        double d1, d2;
        a = normalize(a);
        b = normalize(b);
        d1 = a-b;
        d2 = 2*3.1415926 - fabs(d1);
        if(d1 > 0)  d2 *= -1.0;

        if(fabs(d1) < fabs(d2))
            return d1;
        else
            return d2;
    }

    void drawMarker(tf::Stamped<tf::Pose>& tf_pose, ros::Publisher& pub)
    {
        geometry_msgs::PoseStamped  current_pose;
        tf::poseStampedTFToMsg(tf_pose, current_pose);

        visualization_msgs::Marker points;
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns  = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose = current_pose.pose;
        points.id = 2;
        points.type = visualization_msgs::Marker::SPHERE;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.color.r = 1.0f;
        points.color.a = 1.0;
        pub.publish(points);
    }

    void drawLine(geometry_msgs::Point start, geometry_msgs::Point end,
                  double& a, double& b, double& c)
    {
        visualization_msgs::Marker points;
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns  = "lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 2;
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.color.b = 1.0f;
        points.color.a = 1.0;
        for (uint32_t i = 0; i < 100; ++i)
        {
            double k = - a/b;
            double bias = -c/b;
            double y = k * (start.x + i) + bias;

            geometry_msgs::Point p;
            p.x = start.x + i;
            p.y = y;
            p.z = 0;
            points.points.push_back(p);
        }
        marker_pub3.publish(points);
    }

    // let the linear equation be "ax + by + c = 0"
    // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
    bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double& a, double& b, double& c)
    {
      //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
      double sub_x = fabs(start.x - end.x);
      double sub_y = fabs(start.y - end.y);
      double error = pow(10, -5);  // 0.00001

      if (sub_x < error && sub_y < error)
      {
        ROS_INFO("two points are the same point!!");
        return false;
      }

      a = end.y - start.y;
      b = (-1) * (end.x - start.x);
      c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

      return true;
    }

    double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c)
    {
      double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

      return d;
    }

private:
    tf::TransformListener* tf_;
    boost::mutex configuration_mutex_;
    dynamic_reconfigure::Server<SimpleLocalPlannerConfig> *dsrv_;
    simple_local_planner::SimpleLocalPlannerConfig default_config_;

    double K_x_, K_y_, K_rot_, tolerance_trans_, tolerance_rot_;
    double tolerance_timeout_;
    double max_vel_lin_, max_vel_th_;
    double min_vel_lin_, min_vel_th_;
    double min_in_place_vel_th_, in_place_trans_vel_;
    bool holonomic_;

    bool is_goal_reach_;
    std::string p_robot_base_frame_;
    std::string p_global_frame_;

    boost::mutex odom_lock_;
    ros::Subscriber odom_sub_;

    double trans_stopped_velocity_, rot_stopped_velocity_;
    ros::Time goal_reached_time_;
    unsigned int current_waypoint_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    //base_local_planner::TrajectoryPlannerROS collision_planner_;
    int samples_;
    int steps_;

    Smoother sm_vx;
    Smoother sm_vy;
    Smoother sm_wz;
    geometry_msgs::Twist last_vel;
    ros::Publisher marker_pub1,marker_pub2,marker_pub3,marker_pub4;

    double ld;
    double ld_A,ld_B,ld_C;
    PID pid_vx_;
    PID pid_vy_;
    PID pid_wz_;
};
}

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlanner, nav_core::BaseLocalPlanner)
