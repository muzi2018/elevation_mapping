#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grid_map_msgs/GridMap.h" 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Callback function to process the received message
void elevationMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  // Check if the "elevation" layer exists
  std::string elevationLayer = "elevation";
  auto it = std::find(msg->layers.begin(), msg->layers.end(), elevationLayer);
  if (it != msg->layers.end()) {

    int layer_index = std::distance(msg->layers.begin(), it);

    const std_msgs::Float32MultiArray& elevation_layer = msg->data[layer_index];
    const std::vector<float>& elevation_data = elevation_layer.data;

    // Ensure that we have data
    if (elevation_data.empty()) {
      ROS_WARN("Elevation data is empty.");
      return;
    }

    // Determine the grid dimensions
    if (elevation_layer.layout.dim.size() < 2) {
      ROS_ERROR("Elevation data layout does not contain enough dimensions.");
      return;
    }
    size_t rows = elevation_layer.layout.dim[0].size; // 60
    size_t cols = elevation_layer.layout.dim[1].size; // 60

    // Prepare rotation matrix from the map's orientation, the orientation of the map in the world frame.
    tf::Quaternion q(
      msg->info.pose.orientation.x,
      msg->info.pose.orientation.y,
      msg->info.pose.orientation.z,
      msg->info.pose.orientation.w
    );
    tf::Matrix3x3 rot_matrix(q);

        // Iterate over the elevation data
    for (size_t row = 0; row < rows; ++row) {
      for (size_t col = 0; col < cols; ++col) {
        size_t index = row * cols + col;
        if (index >= elevation_data.size()) continue;
        float elevation = elevation_data[index];

        // Check for NaN values
        if (std::isnan(elevation)) {
          continue;
        }

        // Compute the position in the map frame
        double x = (col + 0.5) * msg->info.resolution - (msg->info.length_x / 2.0);
        // std::cout << "msg->info.resolution = " << msg->info.resolution << std::endl;
        // std::cout << "msg->info.length_x = " << msg->info.length_x << std::endl; 
        // std::cout << "msg->info.length_y = " << msg->info.length_y << std::endl;       
        double y = (row + 0.5) * msg->info.resolution - (msg->info.length_y / 2.0);

        // Apply rotation and translation
        tf::Vector3 position_local(x, y, 0);
        tf::Vector3 position_rotated = rot_matrix * position_local; // align with the world frame
        double pos_x = position_rotated.x() + msg->info.pose.position.x;
        double pos_y = position_rotated.y() + msg->info.pose.position.y;

        // Output the elevation data
        ROS_INFO("Cell (%zu, %zu): World Position (%f, %f), Elevation: %f",
                 row, col, pos_x, pos_y, elevation);
      }
    }
  } else {
    ROS_WARN("Elevation layer not found in the map");
  }

    // std::cout << "*** elevationMapCallback ***" << std::endl;
    // std::cout << "rows = " << rows << std::endl;
    // std::cout << "cols = " << cols << std::endl;
    // std::cout << "num_cells: " << num_cells << std::endl;
}






int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node
  ros::init(argc, argv, "check_map");
  ros::NodeHandle nh;

  // Subscribe to the topic "chatter"
  ros::Subscriber sub = nh.subscribe("/elevation_mapping/elevation_map", 1000, elevationMapCallback);

  // Enter a loop, pumping callbacks
  ros::spin();

  return 0;
}
