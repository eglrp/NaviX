#include "map_server/map_saver.h"

void MapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
           map->info.width,
           map->info.height,
           map->info.resolution);


  std::string mapdatafile = mapname_ + ".pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
          map->info.resolution, map->info.width, map->info.height);
  for(unsigned int y = 0; y < map->info.height; y++) {
    for(unsigned int x = 0; x < map->info.width; x++) {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      if (map->data[i] == 0) { //occ [0,0.1)
        fputc(254, out);
      } else if (map->data[i] == +100) { //occ (0.65,1]
        fputc(000, out);
      } else { //occ [0.1,0.65]
        fputc(205, out);
      }
    }
  }
  fclose(out);

  std::string mapmetadatafile = mapname_ + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
/*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
*/
  geometry_msgs::Quaternion orientation = map->info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
  ));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

  fclose(yaml);

  ROS_INFO("Done\n");
  saved_map_ = true;
}

int MapGenerator::run(void)
{
  std::string mapname = "map";
  map_sub_ = n_.subscribe("map", 1, &MapGenerator::mapCallback, this);
  ros::Rate rate(10);
  while(!saved_map_ && ros::ok())
  {
    rate.sleep();
  }

  return 0;
}
