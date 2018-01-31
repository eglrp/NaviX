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

namespace simple_local_planner {

class SimpleLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        ROS_INFO("LASER");
    }

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

        node_private.param("k_trans", K_trans_, 1.5);
        node_private.param("k_rot", K_rot_, 2.0);

        node_private.param("tolerance_trans", tolerance_trans_, 0.1);
        node_private.param("tolerance_rot", tolerance_rot_, 0.1);
        node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

        node_private.param("holonomic", holonomic_, true);

        node_private.param("samples", samples_, 1);
        node_private.param("steps", steps_, 3);

        node_private.param("max_vel_lin", max_vel_lin_, 0.7);
        node_private.param("min_vel_lin", min_vel_lin_, 0.05);

        node_private.param("max_vel_th", max_vel_th_, 1.0);
        node_private.param("min_vel_th", min_vel_th_, 0.2);

        node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.1);
        node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.1);

        node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
        node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

        node_private.param("robot_base_frame", p_robot_base_frame_, std::string("base_link"));
        node_private.param("global_frame", p_global_frame_, std::string("map"));

        dsrv_ = new dynamic_reconfigure::Server<SimpleLocalPlannerConfig>(node_private);
        dynamic_reconfigure::Server<SimpleLocalPlannerConfig>::CallbackType cb = boost::bind(&SimpleLocalPlanner::reconfigure, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void reconfigure(SimpleLocalPlannerConfig &config, uint32_t level)
     {
        boost::mutex::scoped_lock l(configuration_mutex_);

        K_trans_ = config.k_trans;
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
        if(!transformGlobalPlan(*tf_, global_plan, p_global_frame_, global_plan_))
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

        //get the current pose of the robot in the fixed frame
        tf::Stamped<tf::Pose> robot_pose;
        if(!this->getRobotPose(robot_pose))
        {
          ROS_ERROR("Can't get robot pose");
          geometry_msgs::Twist empty_twist;
          cmd_vel = empty_twist;
          return false;
        }

        //we want to compute a velocity command based on our current waypoint
        tf::Stamped<tf::Pose> target_pose;
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);

        //get the difference between the two poses
        geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
        //ROS_INFO("HectorPathFollower: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);
        geometry_msgs::Twist limit_vel = limitTwist(diff);

        geometry_msgs::Twist test_vel = limit_vel;
        bool legal_traj = true;//collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, true);

        double scaling_factor = 1.0;
        double ds = scaling_factor / samples_;

        //let's make sure that the velocity command is legal... and if not, scale down
        if(!legal_traj)
        {
          for(int i = 0; i < samples_; ++i)
          {
            test_vel.linear.x = limit_vel.linear.x * scaling_factor;
            test_vel.linear.y = limit_vel.linear.y * scaling_factor;
            test_vel.angular.z = limit_vel.angular.z * scaling_factor;
            test_vel = limitTwist(test_vel);
            //test time
            scaling_factor -= ds;
          }
        }

        if(!legal_traj)
        {
          ROS_ERROR("Not legal (%.2f, %.2f, %.2f)",
                    limit_vel.linear.x,
                    limit_vel.linear.y,
                    limit_vel.angular.z);
          geometry_msgs::Twist empty_twist;
          cmd_vel = empty_twist;
          return false;
        }

        //if it is legal... we'll pass it on
        cmd_vel = test_vel;

        is_goal_reach_ = false;
        while(fabs(diff.linear.x)  <=  tolerance_trans_ &&
              fabs(diff.linear.y)  <=  tolerance_trans_ &&
              fabs(diff.angular.z) <=  tolerance_rot_)
        {
          if(current_waypoint_ < global_plan_.size()-1)
          {
            is_goal_reach_ = false;
            current_waypoint_ += steps_;
            if((current_waypoint_+ steps_) > global_plan_.size())
            {
                current_waypoint_ = global_plan_.size()-1;
            }

            ROS_INFO(" Path has run %3.1f%%. size=%d current=%d",
                     current_waypoint_ * 100.0 / (global_plan_.size()-1),
                     global_plan_.size()-1,
                     current_waypoint_);
            try
            {
                tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
                diff = diff2D(target_pose, robot_pose);
            }
            catch(std::exception& e)
            {
                std::cout<<e.what()<<std::endl;
            }
            ROS_INFO("#################################");
          }
          else
          {
            ROS_INFO("Reached goal: %d", current_waypoint_);
            geometry_msgs::Twist empty_twist;
            cmd_vel = empty_twist;
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

    geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
    {
      geometry_msgs::Twist res;
      tf::Pose diff = pose2.inverse() * pose1;
      res.linear.x = diff.getOrigin().x();
      res.linear.y = diff.getOrigin().y();
      res.angular.z = tf::getYaw(diff.getRotation());

      if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
          return res;

      //in the case that we're not rotating to our goal position and we have a non-holonomic robot
      //we'll need to command a rotational velocity that will help us reach our desired heading

      //we want to compute a goal based on the heading difference between our pose and the target
      double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
          pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

      //we'll also check if we can move more effectively backwards
      double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
          pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

      //check if its faster to just back up
      if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
        ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
        yaw_diff = neg_yaw_diff;
      }

      //compute the desired quaterion
      tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
      tf::Quaternion rot = pose2.getRotation() * rot_diff;
      tf::Pose new_pose = pose1;
      new_pose.setRotation(rot);

      diff = pose2.inverse() * new_pose;
      res.linear.x = diff.getOrigin().x();
      res.linear.y = diff.getOrigin().y();
      res.angular.z = tf::getYaw(diff.getRotation());
      return res;
    }

    geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist)
    {
      geometry_msgs::Twist res = twist;
      res.linear.x *= K_trans_;
      if(!holonomic_)
        res.linear.y = 0.0;
      else
        res.linear.y *= K_trans_;
      res.angular.z *= K_rot_;

      //make sure to bound things by our velocity limits
      double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
      double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
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
      if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_)
      {
        if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
        res.linear.x = 0.0;
        res.linear.y = 0.0;
        ROS_INFO("Angular command %f", res.angular.z);
      }

      return res;
    }

    bool transformGlobalPlan(const tf::TransformListener& tf,
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

    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
    {
      global_pose.setIdentity();
      tf::Stamped<tf::Pose> robot_pose;
      robot_pose.setIdentity();
      robot_pose.frame_id_ = p_robot_base_frame_;
      robot_pose.stamp_ = ros::Time(0);
      ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

      //get the global pose of the robot
      try{
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
//      if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
//        ROS_WARN_THROTTLE(1.0, "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
//            current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance_);
//        return false;
//      }
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

    tf::TransformListener* tf_;

    boost::mutex configuration_mutex_;
    dynamic_reconfigure::Server<SimpleLocalPlannerConfig> *dsrv_;
    simple_local_planner::SimpleLocalPlannerConfig default_config_;

    double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
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
};
};

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlanner, nav_core::BaseLocalPlanner)
