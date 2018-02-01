#ifndef __JOY__
#define __JOY__

#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

namespace JOY
{
    enum JOY_STATUS
    {
        PRESSED = 1,
        RELEASE = 0
    };

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
}
///\brief Opens, reads from and publishes joystick events
class Joystick
{
private:
  ros::NodeHandle nh_;
  bool open_;
  bool sticky_buttons_;
  bool default_trig_val_;
  std::string joy_dev_;
  std::string joy_dev_name_;
  double deadzone_;
  double autorepeat_rate_;   // in Hz.  0 for no repeat.
  double coalesce_interval_; // Defaults to 100 Hz rate limit.
  int event_count_;
  int pub_count_;
  ros::Publisher pub_;
  double lastDiagTime_;

  /*! \brief Returns the device path of the first joystick that matches joy_name.
   *         If no match is found, an empty string is returned.
   */
  std::string get_dev_by_joy_name(const std::string& joy_name);
  
public:
  Joystick() : nh_()
  {}
  
  ///\brief Opens joystick port, reads from port and publishes while node is active
  int run();
};

#endif
