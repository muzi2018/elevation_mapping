#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grid_map_msgs/GridMap.h" 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_ros/grid_map_ros.hpp>
// Callback function to process the received message
void elevationMapCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);

  std::cout << "elevationMapCallback......." << std::endl;

  if (!map.exists("elevation")) {
    ROS_WARN("Elevation layer not found in the map.");
    return;
  }

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const float elevation = map.at("elevation", *iterator);

    grid_map::Position position;
    map.getPosition(*iterator, position);
    if (elevation <= -0.07)
    {
      ROS_INFO("Position (%f, %f): Elevation %f", position.x(), position.y(), elevation);
    }
    
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/elevation_mapping/elevation_map", 1, elevationMapCallback);

  ros::spin();
  return 0;
}
