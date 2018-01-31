#include "urg_node/urg_node_driver.h"

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_node");

  urg_node::UrgNode node;
  node.run();

  ros::spin();

  return 0;
}
