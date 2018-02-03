#include "joy/joy.h"


/*! \brief Returns the device path of the first joystick that matches joy_name.
*         If no match is found, an empty string is returned.
*/
std::string Joystick::get_dev_by_joy_name(const std::string& joy_name)
{
    const char path[] = "/dev/input"; // no trailing / here
    struct dirent *entry;
    struct stat stat_buf;

    DIR *dev_dir = opendir(path);
    if (dev_dir == NULL)
    {
      ROS_ERROR("Couldn't open %s. Error %i: %s.", path, errno, strerror(errno));
      return "";
    }

    while ((entry = readdir(dev_dir)) != NULL)
    {
      // filter entries
      if (strncmp(entry->d_name, "js", 2) != 0) // skip device if it's not a joystick
        continue;
      std::string current_path = std::string(path) + "/" + entry->d_name;
      if (stat(current_path.c_str(), &stat_buf) == -1)
        continue;
      if (!S_ISCHR(stat_buf.st_mode)) // input devices are character devices, skip other
        continue;

      // get joystick name
      int joy_fd = open(current_path.c_str(), O_RDONLY);
      if (joy_fd == -1)
        continue;

      char current_joy_name[128];
      if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0)
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));

      close(joy_fd);

      ROS_INFO("Found joystick: %s (%s).", current_joy_name, current_path.c_str());

      if (strcmp(current_joy_name, joy_name.c_str()) == 0)
      {
          closedir(dev_dir);
          return current_path;
      }
    }

    closedir(dev_dir);
    return "";
}

///\brief Opens joystick port, reads from port and publishes while node is active
int Joystick::run()
{
    // Parameters
    ros::NodeHandle nh_param("~");
    pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
    nh_param.param<std::string>("dev", joy_dev_, "/dev/input/js0");
    nh_param.param<std::string>("dev_name", joy_dev_name_, "");
    nh_param.param<double>("deadzone", deadzone_, 0.05);
    nh_param.param<double>("autorepeat_rate", autorepeat_rate_, 0);
    nh_param.param<double>("coalesce_interval", coalesce_interval_, 0.001);
    nh_param.param<bool>("default_trig_val",default_trig_val_,false);
    nh_param.param<bool>("sticky_buttons", sticky_buttons_, false);

    // Checks on parameters
    if (!joy_dev_name_.empty())
    {
        std::string joy_dev_path = get_dev_by_joy_name(joy_dev_name_);
        if (joy_dev_path.empty())
            ROS_ERROR("Couldn't find a joystick with name %s. Falling back to default device.", joy_dev_name_.c_str());
        else
        {
            ROS_INFO("Using %s as joystick device.", joy_dev_path.c_str());
            joy_dev_ = joy_dev_path;
        }
    }

    if (autorepeat_rate_ > 1 / coalesce_interval_)
      ROS_WARN("joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1/coalesce_interval_);

    if (deadzone_ >= 1)
    {
      ROS_WARN("joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone by 32767, but this behavior is deprecated so you need to update your launch file.");
      deadzone_ /= 32767;
    }

    if (deadzone_ > 0.9)
    {
      ROS_WARN("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
      deadzone_ = 0.9;
    }

    if (deadzone_ < 0)
    {
      ROS_WARN("joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
      deadzone_ = 0;
    }

    if (autorepeat_rate_ < 0)
    {
      ROS_WARN("joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
      autorepeat_rate_ = 0;
    }

    if (coalesce_interval_ < 0)
    {
      ROS_WARN("joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
      coalesce_interval_ = 0;
    }

    // Parameter conversions
    double autorepeat_interval = 1 / autorepeat_rate_;
    double scale = -1. / (1. - deadzone_) / 32767.;
    double unscaled_deadzone = 32767. * deadzone_;

    js_event event;
    struct timeval tv;
    fd_set set;
    int joy_fd;
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = ros::Time::now().toSec();

    // Big while loop opens, publishes
    while (nh_.ok())
    {
      boost::this_thread::interruption_point();
      open_ = false;
      bool first_fault = true;
      while (nh_.ok())
      {
        boost::this_thread::interruption_point();
        ros::spinOnce();
        if (!nh_.ok())
            break;
        joy_fd = open(joy_dev_.c_str(), O_RDWR);
        if (joy_fd != -1)
        {
            // There seems to be a bug in the driver or something where the
            // initial events that are to define the initial state of the
            // joystick are not the values of the joystick when it was opened
            // but rather the values of the joystick when it was last closed.
            // Opening then closing and opening again is a hack to get more
            // accurate initial state data.
            close(joy_fd);
            joy_fd = open(joy_dev_.c_str(), O_RDWR);
        }
        if (joy_fd != -1)
            break;
        if (first_fault)
        {
            ROS_ERROR("Couldn't open joystick %s. Will retry every second.", joy_dev_.c_str());
            first_fault = false;
        }
        usleep(500000L);
      }

      open_ = true;
      printf("[JOYSTICK] Joy open normally.\r\n");
      fflush(stdout);

      bool tv_set = false;
      bool publication_pending = false;
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      sensor_msgs::Joy joy_msg; // Here because we want to reset it on device close.
      double val; //Temporary variable to hold event values
      sensor_msgs::Joy last_published_joy_msg; // used for sticky buttons option
      sensor_msgs::Joy sticky_buttons_joy_msg; // used for sticky buttons option
      while (nh_.ok())
      {
        boost::this_thread::interruption_point();
        ros::spinOnce();

        bool publish_now = false;
        bool publish_soon = false;
        FD_ZERO(&set);
        FD_SET(joy_fd, &set);

        int select_out = select(joy_fd+1, &set, NULL, NULL, &tv);
        if (select_out == -1)
        {
          tv.tv_sec = 0;
          tv.tv_usec = 0;
          continue;
        }

        if (FD_ISSET(joy_fd, &set))
        {
          if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
            break; // Joystick is probably closed. Definitely occurs.

          joy_msg.header.stamp = ros::Time().now();
          event_count_++;
          switch(event.type)
          {
          case JS_EVENT_BUTTON:
          case JS_EVENT_BUTTON | JS_EVENT_INIT:
            if(event.number >= joy_msg.buttons.size())
            {
              int old_size = joy_msg.buttons.size();
              joy_msg.buttons.resize(event.number+1);
              last_published_joy_msg.buttons.resize(event.number+1);
              sticky_buttons_joy_msg.buttons.resize(event.number+1);
              for(unsigned int i=old_size;i<joy_msg.buttons.size();i++)
              {
                joy_msg.buttons[i] = 0.0;
                last_published_joy_msg.buttons[i] = 0.0;
                sticky_buttons_joy_msg.buttons[i] = 0.0;
              }
            }
            joy_msg.buttons[event.number] = (event.value ? 1 : 0);
            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            if (!(event.type & JS_EVENT_INIT))
              publish_now = true;
            else
              publish_soon = true;
            break;
          case JS_EVENT_AXIS:
          case JS_EVENT_AXIS | JS_EVENT_INIT:
            val = event.value;
            if(event.number >= joy_msg.axes.size())
            {
              int old_size = joy_msg.axes.size();
              joy_msg.axes.resize(event.number+1);
              last_published_joy_msg.axes.resize(event.number+1);
              sticky_buttons_joy_msg.axes.resize(event.number+1);
              for(unsigned int i=old_size;i<joy_msg.axes.size();i++)
              {
                joy_msg.axes[i] = 0.0;
                last_published_joy_msg.axes[i] = 0.0;
                sticky_buttons_joy_msg.axes[i] = 0.0;
              }
            }
        if(default_trig_val_)
        {
            // Allows deadzone to be "smooth"
            if (val > unscaled_deadzone)
                val -= unscaled_deadzone;
            else if (val < -unscaled_deadzone)
                val += unscaled_deadzone;
            else
                val = 0;
            joy_msg.axes[event.number] = val * scale;
            // Will wait a bit before sending to try to combine events.
            publish_soon = true;
            break;
        }
        else
        {
          if (!(event.type & JS_EVENT_INIT))
          {
            val = event.value;
            if(val > unscaled_deadzone)
              val -= unscaled_deadzone;
            else if(val < -unscaled_deadzone)
              val += unscaled_deadzone;
            else
              val=0;
            joy_msg.axes[event.number]= val * scale;
          }

          publish_soon = true;
          break;
          default:
          ROS_WARN("joy_node: Unknown event type. Please file a ticket. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
          break;
        }
      }
    }
    else if (tv_set) // Assume that the timer has expired.
      publish_now = true;

    if (publish_now)
    {
      if (sticky_buttons_ == true)
      {
        // cycle through buttons
        for (size_t i = 0; i < joy_msg.buttons.size(); i++)
        {
            // change button state only on transition from 0 to 1
            if (joy_msg.buttons[i] == 1 && last_published_joy_msg.buttons[i] == 0)
            {
                sticky_buttons_joy_msg.buttons[i] = sticky_buttons_joy_msg.buttons[i] ? 0 : 1;
            }
            else
            {
            // do not change the message sate
            //sticky_buttons_joy_msg.buttons[i] = sticky_buttons_joy_msg.buttons[i] ? 0 : 1;
            }
        }
        // update last published message
        last_published_joy_msg = joy_msg;
        // fill rest of sticky_buttons_joy_msg (time stamps, axes, etc)
        sticky_buttons_joy_msg.header.stamp.nsec = joy_msg.header.stamp.nsec;
        sticky_buttons_joy_msg.header.stamp.sec  = joy_msg.header.stamp.sec;
        sticky_buttons_joy_msg.header.frame_id   = joy_msg.header.frame_id;
        for(size_t i=0; i < joy_msg.axes.size(); i++)
        {
            sticky_buttons_joy_msg.axes[i] = joy_msg.axes[i];
        }
        pub_.publish(sticky_buttons_joy_msg);
      }
      else
      {
        pub_.publish(joy_msg);
      }

      publish_now = false;
      tv_set = false;
      publication_pending = false;
      publish_soon = false;
      pub_count_++;
    }

    // If an axis event occurred, start a timer to combine with other
    // events.
    if (!publication_pending && publish_soon)
    {
      tv.tv_sec = trunc(coalesce_interval_);
      tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
      publication_pending = true;
      tv_set = true;
      //ROS_INFO("Pub pending...");
    }

    // If nothing is going on, start a timer to do autorepeat.
    if (!tv_set && autorepeat_rate_ > 0)
    {
      tv.tv_sec = trunc(autorepeat_interval);
      tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
      tv_set = true;
      //ROS_INFO("Autorepeat pending... %li %li", tv.tv_sec, tv.tv_usec);
    }

    if (!tv_set)
    {
      tv.tv_sec = 1;
      tv.tv_usec = 0;
    }

    } // End of joystick open loop.

      close(joy_fd);
      ros::spinOnce();
      if (nh_.ok())
        ROS_ERROR("Connection to joystick device lost unexpectedly. Will reopen.");
    }
    return 0;
}
