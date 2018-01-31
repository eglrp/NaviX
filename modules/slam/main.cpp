#include <ros/ros.h>

#include "gmapping/openslam/slam_gmapping.h"
#include "amcl/amcl.h"

boost::shared_ptr<AmclNode> amcl_node_ptr;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_gmapping");


  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new AmclNode());


  //SlamGMapping gn;

  //gn.startLiveSlam();
  ros::spin();
  ros::shutdown();
  amcl_node_ptr.reset();
  return(0);
}

