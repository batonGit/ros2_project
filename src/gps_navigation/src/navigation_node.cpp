#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class GPSNavigationNode : public rclcpp::Node
{
public:
  GPSNavigationNode() : Node("gps_navigation_node"), autonomous_mode_(false)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    destination_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10,
      std::bind(&GPSNavigationNode::destination_callback, this, std::placeholders::_1));

    robot_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/robot/gps/fix", 10,
      std::bind(&GPSNavigationNode::robot_gps_callback, this, std::placeholders::_1));

    destination_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/destination_id", 10,
      std::bind(&GPSNavigationNode::destination_id_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&GPSNavigationNode::lidar_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "GPS Navigation Node initialized");
  }

private:
  void destination_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    current_destination_lat_ = msg->latitude;
    current_destination_lon_ = msg->longitude;
    RCLCPP_INFO(this->get_logger(), "Received destination GPS: Lat: %.6f, Lon: %.6f",
                current_destination_lat_, current_destination_lon_);

    if (autonomous_mode_) {
      update_navigation();
    }
  }

  void robot_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (initial_lat_ == 0.0 && initial_lon_ == 0.0) {
      initial_lat_ = msg->latitude;
      initial_lon_ = msg->longitude;
    }

    robot_lat_ = msg->latitude;
    robot_lon_ = msg->longitude;
    RCLCPP_INFO(this->get_logger(), "Robot GPS: Lat: %.6f, Lon: %.6f", robot_lat_, robot_lon_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    double delta_lat = robot_lat_ - initial_lat_;
    double delta_lon = robot_lon_ - initial_lon_;
    transform.transform.translation.x = delta_lat * 111320.0;
    transform.transform.translation.y = delta_lon * 111320.0 * cos(robot_lat_ * M_PI / 180.0);
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);

    if (autonomous_mode_) {
      update_navigation();
    }
  }

  void destination_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int destination_id = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received destination ID: %d", destination_id);

    if (destination_id == 1) {
      // Используем координаты из /gps/fix, которые обновляются в destination_callback
      if (current_destination_lat_ != 0.0 && current_destination_lon_ != 0.0) {
        RCLCPP_INFO(this->get_logger(), "Setting destination from /gps/fix: Lat: %.6f, Lon: %.6f",
                    current_destination_lat_, current_destination_lon_);
        autonomous_mode_ = true;
        update_navigation();
      } else {
        RCLCPP_WARN(this->get_logger(), "Destination coordinates not yet received from /gps/fix");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid destination ID: %d", destination_id);
    }
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    obstacle_detected_ = false;
    int front_start_idx = static_cast<int>((330 * M_PI / 180 - msg->angle_min) / msg->angle_increment);
    int front_end_idx = static_cast<int>((30 * M_PI / 180 - msg->angle_min) / msg->angle_increment);

    if (front_start_idx < 0) front_start_idx += msg->ranges.size();
    if (front_end_idx >= static_cast<int>(msg->ranges.size())) front_end_idx -= msg->ranges.size();

    for (int i = front_start_idx; i != front_end_idx; i = (i + 1) % msg->ranges.size()) {
      if (msg->ranges[i] > msg->range_min &&
          msg->ranges[i] < msg->range_max &&
          msg->ranges[i] < min_obstacle_distance_) {
        obstacle_detected_ = true;
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f meters!", msg->ranges[i]);
        break;
      }
    }
  }

  void update_navigation()
  {
    if (current_destination_lat_ == 0.0 || robot_lat_ == 0.0) {
      return;
    }

    double distance = calculate_distance(robot_lat_, robot_lon_,
                                         current_destination_lat_, current_destination_lon_);
    double bearing = calculate_bearing(robot_lat_, robot_lon_,
                                       current_destination_lat_, current_destination_lon_);

    RCLCPP_INFO(this->get_logger(), "Distance to destination: %.2f meters, Bearing: %.2f degrees",
                distance, bearing);

    save_map_point(robot_lat_, robot_lon_);

    auto cmd_msg = geometry_msgs::msg::Twist();

    if (obstacle_detected_) {
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.5;
      RCLCPP_WARN(this->get_logger(), "Avoiding obstacle...");
    }
    else if (distance > 1.0) {
      cmd_msg.linear.x = 0.2;
      cmd_msg.angular.z = calculate_steering(bearing);
    }
    else {
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Destination reached!");
      autonomous_mode_ = false;
      save_map();
    }

    cmd_vel_pub_->publish(cmd_msg);
  }

  void save_map_point(double lat, double lon)
  {
    map_points_.push_back(std::make_pair(lat, lon));
  }

  void save_map()
  {
    std::string map_path = "/home/robot/maps/route_map.txt";
    std::ofstream map_file(map_path);

    if (map_file.is_open()) {
      for (const auto& point : map_points_) {
        map_file << point.first << "," << point.second << "\n";
      }
      map_file.close();
      RCLCPP_INFO(this->get_logger(), "Map saved to %s", map_path.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map");
    }

    map_points_.clear();
  }

  double calculate_distance(double lat1, double lon1, double lat2, double lon2)
  {
    const double R = 6371000.0;
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return R * c;
  }

  double calculate_bearing(double lat1, double lon1, double lat2, double lon2)
  {
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;

    double dlon = lon2_rad - lon1_rad;

    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon);

    double bearing_rad = atan2(y, x);
    double bearing_deg = bearing_rad * 180.0 / M_PI;

    if (bearing_deg < 0) {
      bearing_deg += 360.0;
    }

    return bearing_deg;
  }

  double calculate_steering(double target_bearing)
  {
    double bearing_diff = target_bearing - 0.0;

    if (bearing_diff > 180) {
      bearing_diff -= 360;
    } else if (bearing_diff < -180) {
      bearing_diff += 360;
    }

    return std::clamp(bearing_diff / 90.0, -1.0, 1.0);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr destination_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robot_gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr destination_id_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double initial_lat_ = 0.0;
  double initial_lon_ = 0.0;
  double current_destination_lat_ = 0.0;
  double current_destination_lon_ = 0.0;
  double robot_lat_ = 0.0;
  double robot_lon_ = 0.0;
  bool autonomous_mode_;
  bool obstacle_detected_ = false;
  float min_obstacle_distance_ = 0.5;

  std::vector<std::pair<double, double>> map_points_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPSNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
