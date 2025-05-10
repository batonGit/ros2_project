#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/int32.hpp" // Removed: Not needed after removing /destination_id sub
#include "std_msgs/msg/bool.hpp"   // New: for mode control
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
// Removed fstream, iostream, string, vector for GPS log
#include <memory>
#include <algorithm>
#include <limits>
// Added iomanip for setprecision (if needed, though logging removed)
// #include <iomanip>

class GPSNavigationNode : public rclcpp::Node
{
public:
    GPSNavigationNode() : Node("gps_navigation_node"),
                          tf_buffer_(this->get_clock()), // tf_buffer_ initialized before gps_navigation_active_
                          gps_navigation_active_(false), // Initialize here
                          has_gps_target_(false),       // Initialize here
                          robot_lat_(0.0),              // Initialize here
                          robot_lon_(0.0),              // Initialize here
                          current_destination_lat_(0.0),// Initialize here
                          current_destination_lon_(0.0),// Initialize here
                          obstacle_detected_(false),    // Initialize here
                          min_obstacle_distance_(0.5),  // Initialize here
                          last_cmd_vel_was_zero_(true)  // Initialize here
    {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "TF Listener initialized");

        // REMOVED: Subscription to /gps/fix (destination 1)
        // REMOVED: Subscription to /destination_id

        // Subscription to robot's current GPS location
        robot_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/robot/gps/fix", 10,
            std::bind(&GPSNavigationNode::robot_gps_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot/gps/fix");

        // NEW: Subscription to target GPS coordinates from manager
        gps_target_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_target", 10,
            std::bind(&GPSNavigationNode::gps_target_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /gps_target");

        // NEW: Subscription to mode control from manager
        gps_mode_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/gps_mode_control", 10,
            std::bind(&GPSNavigationNode::gps_mode_control_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /gps_mode_control");


        // Assuming /scan is published in base_link or lidar_frame, connected to base_link via TF
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GPSNavigationNode::lidar_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /scan");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing to /cmd_vel");

        // Timer to trigger navigation updates periodically
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Update frequency (e.g., 10 Hz)
            std::bind(&GPSNavigationNode::update_navigation, this)
        );
        RCLCPP_INFO(this->get_logger(), "Update timer created");

        RCLCPP_INFO(this->get_logger(), "GPS Navigation Node initialized (waiting for manager commands)");
    }

private:
    // REMOVED: destination_callback

    void robot_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Store robot's latest GPS.
        robot_lat_ = msg->latitude;
        robot_lon_ = msg->longitude;
        // RCLCPP_INFO(this->get_logger(), "Robot GPS: Lat: %.6f, Lon: %.6f", robot_lat_, robot_lon_); // Too frequent
    }

    // NEW: Callback for receiving target GPS from manager
    void gps_target_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        current_destination_lat_ = msg->latitude;
        current_destination_lon_ = msg->longitude;
        has_gps_target_ = true; // Flag to indicate we have a valid target
        RCLCPP_INFO(this->get_logger(), "Received new GPS target: Lat: %.6f, Lon: %.6f",
                    current_destination_lat_, current_destination_lon_);

        // If navigation is active, the timer will pick up the new target.
        // If not active, manager needs to set gps_mode_control_ to true.
    }

    // NEW: Callback for mode control from manager
    void gps_mode_control_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        gps_navigation_active_ = msg->data;
        if (gps_navigation_active_) {
            RCLCPP_INFO(this->get_logger(), "GPS Navigation activated by manager.");
            // Note: Navigation will only start if has_gps_target_ is also true
        } else {
            RCLCPP_INFO(this->get_logger(), "GPS Navigation deactivated by manager. Stopping.");
            // Stop robot immediately
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_msg);
            last_cmd_vel_was_zero_ = true;
            has_gps_target_ = false; // Clear target when deactivated
        }
    }

    // REMOVED: destination_id_callback (logic moved to manager)

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Basic obstacle detection - checks a fixed angular sector
        obstacle_detected_ = false;
        float min_range_in_sector = std::numeric_limits<float>::max();

        // Define a sector (e.g., +/- 30 degrees from front)
        double sector_angle_deg = 30.0;
        double sector_angle_rad = sector_angle_deg * M_PI / 180.0;

        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;
        size_t num_ranges = msg->ranges.size();

        // Find indices corresponding to -sector_angle_rad and +sector_angle_rad relative to the Lidar's forward direction (0)
        // Assuming Lidar forward is at angle 0 in its own frame
        // Need to handle wrap-around
        int start_idx_int = static_cast<int>((-sector_angle_rad - angle_min) / angle_increment);
        int end_idx_int = static_cast<int>((sector_angle_rad - angle_min) / angle_increment);

        // Normalize indices to be within [0, num_ranges - 1] using modular arithmetic
        size_t start_idx = (start_idx_int % static_cast<int>(num_ranges) + static_cast<int>(num_ranges)) % static_cast<int>(num_ranges);
        size_t end_idx = (end_idx_int % static_cast<int>(num_ranges) + static_cast<int>(num_ranges)) % static_cast<int>(num_ranges);


        // Iterate through the ranges in the front sector
        if (start_idx <= end_idx) {
            for (size_t i = start_idx; i <= end_idx; ++i) { // Use size_t
                float range = msg->ranges[i];
                if (!std::isnan(range) && range > msg->range_min && range < msg->range_max) {
                    if (range < min_obstacle_distance_) {
                        obstacle_detected_ = true;
                        min_range_in_sector = std::min(min_range_in_sector, range);
                    }
                }
            }
        } else { // Handle wrap-around case (e.g., sector from 330 to 30 deg)
             for (size_t i = start_idx; i < num_ranges; ++i) { // Use size_t
                 float range = msg->ranges[i];
                 if (!std::isnan(range) && range > msg->range_min && range < msg->range_max) {
                    if (range < min_obstacle_distance_) {
                        obstacle_detected_ = true;
                        min_range_in_sector = std::min(min_range_in_sector, range);
                    }
                 }
             }
             for (size_t i = 0; i <= end_idx; ++i) { // Use size_t
                 float range = msg->ranges[i];
                  if (!std::isnan(range) && range > msg->range_min && range < msg->range_max) {
                    if (range < min_obstacle_distance_) {
                        obstacle_detected_ = true;
                        min_range_in_sector = std::min(min_range_in_sector, range);
                    }
                 }
             }
        }

        if (obstacle_detected_) {
             // RCLCPP_WARN(this->get_logger(), "Obstacle detected within %.1f deg sector at %.2f meters!", // Too frequent
             //            sector_angle_deg * 2, min_range_in_sector);
        }
    }


    void update_navigation()
    {
        // Only navigate if GPS mode is active AND we have a target AND robot GPS is available
        if (!gps_navigation_active_ || !has_gps_target_ || robot_lat_ == 0.0) {
            // Ensure robot is stopped if navigation is not active
            if (!last_cmd_vel_was_zero_) { // Only publish if the last command wasn't already zero
                 auto cmd_msg = geometry_msgs::msg::Twist();
                 cmd_msg.linear.x = 0.0;
                 cmd_msg.angular.z = 0.0;
                 cmd_vel_pub_->publish(cmd_msg);
                 last_cmd_vel_was_zero_ = true;
                 // RCLCPP_INFO(this->get_logger(), "Navigation not active, published zero Twist."); // Too frequent
            }
            return; // Exit update cycle
        }

        last_cmd_vel_was_zero_ = false; // We are attempting to move


        // --- Get Robot's current pose (position and orientation) from TF ---
        geometry_msgs::msg::TransformStamped transformStamped;
        double robot_current_yaw = 0.0; // Yaw in radians

        try {
            // Look up the transform from the "odom" frame to the "base_link" frame
            // Using Time(0) means get the latest available transform
            transformStamped = tf_buffer_.lookupTransform(
                "odom", "base_link", tf2::TimePointZero);

            // Extract Yaw from the quaternion
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw); // Yaw is in radians

            robot_current_yaw = yaw;

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(
                this->get_logger(), "Could not transform 'odom' to 'base_link' for yaw calculation: %s. Stopping.",
                ex.what());
            // If we can't get the robot's pose, we cannot navigate reliably. Stop the robot.
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_msg);
            last_cmd_vel_was_zero_ = true;
            // On critical TF error, potentially notify manager?
            // For now, just stop and wait for TF to become available again.
            return; // Exit update cycle
        }
        // --- End TF Lookup ---


        // --- GPS Calculations (using robot_lat/lon and current_destination_lat/lon) ---
        double distance = calculate_distance(robot_lat_, robot_lon_,
                                             current_destination_lat_, current_destination_lon_);
        double target_bearing = calculate_bearing(robot_lat_, robot_lon_,
                                                 current_destination_lat_, current_destination_lon_); // Bearing in degrees
        // --- End GPS Calculations ---

        // RCLCPP_INFO(this->get_logger(), // Too frequent
        //              "Dist: %.2f m, Target Bearing: %.2f deg, Robot Yaw: %.2f deg (%.2f rad)",
        //              distance, target_bearing, robot_current_yaw * 180.0/M_PI, robot_current_yaw);


        auto cmd_msg = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {
            // Simple avoidance: Stop and turn in place. Needs improvement for real navigation.
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.5; // Turn rate (e.g., rotate left)
            RCLCPP_WARN(this->get_logger(), "Obstacle detected within %.1f deg sector. Avoiding...", 60.0); // Hardcoded sector angle for log
        }
        else if (distance > 1.0) { // Threshold to consider destination reached (1 meter)
            cmd_msg.linear.x = 0.2; // Fixed forward speed (adjust as needed)
            // Calculate steering based on target bearing and current robot yaw
            cmd_msg.angular.z = calculate_steering(target_bearing, robot_current_yaw);
            // Clamp angular velocity again after steering calculation if calculate_steering doesn't clamp
            cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -1.0, 1.0); // Ensure it's within bounds
        }
        else {
            // Destination reached
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_msg); // Publish zero command
            last_cmd_vel_was_zero_ = true;
            RCLCPP_INFO(this->get_logger(), "GPS Destination reached!");
            gps_navigation_active_ = false; // Stop GPS navigation
            has_gps_target_ = false; // Clear the target
            // TODO: Publish a message to the manager indicating goal reached (optional)
        }

        // Only publish if we are active and not stopped due to reaching goal in this cycle
        if (gps_navigation_active_) {
            cmd_vel_pub_->publish(cmd_msg);
        }
    }

    // REMOVED: log_gps_waypoint
    // REMOVED: save_gps_log

    double calculate_distance(double lat1, double lon1, double lat2, double lon2)
    {
        // Haversine formula code - remains the same
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
        // Bearing formula code - remains the same
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

        // Normalize to 0-360 degrees
        if (bearing_deg < 0) {
            bearing_deg += 360.0;
        }

        return bearing_deg;
    }

    // Steering function using robot's current yaw
    double calculate_steering(double target_bearing_deg, double current_yaw_rad)
    {
        // Convert robot's current yaw from radians (-pi to pi) to degrees (-180 to 180)
        double current_yaw_deg = current_yaw_rad * 180.0 / M_PI;

        // Calculate the difference between target bearing (0-360) and robot's current heading (-180 to 180)
        // Need to handle the wrap-around logic correctly
        double error_deg = target_bearing_deg - current_yaw_deg;

        // Normalize error to the range [-180, 180]
        if (error_deg > 180.0) {
            error_deg -= 360.0;
        } else if (error_deg < -180.0) {
            error_deg += 360.0;
        }

        // RCLCPP_INFO(this->get_logger(), "Steering Error: %.2f degrees", error_deg); // Too frequent


        // Simple Proportional controller
        // Output angular velocity is proportional to the bearing error
        // Scaling factor 1/90.0 means max angular velocity (1.0) is reached at 90 deg error
        double angular_vel = error_deg / 90.0; // Adjust scaling factor (90.0) as needed

        // Clamp the output to avoid excessive spinning - Manager will also clamp, but good practice here too.
        // angular_vel = std::clamp(angular_vel, -1.0, 1.0); // Already clamped in update_navigation

        return angular_vel;
    }

    // Removed initial_lat/lon as they weren't used for nav logic anymore

    // --- Member Variables ---
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robot_gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_target_sub_; // NEW
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_mode_control_sub_; // NEW
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    double current_destination_lat_;
    double current_destination_lon_;
    bool has_gps_target_; // NEW: Flag to indicate if a target has been received

    double robot_lat_;
    double robot_lon_;

    bool gps_navigation_active_; // NEW: Controlled by manager
    bool obstacle_detected_;
    float min_obstacle_distance_; // Obstacle detection threshold in meters

    bool last_cmd_vel_was_zero_; // To avoid spamming zero commands when idle

    // Removed std::vector<std::pair<double, double>> gps_trajectory_points_;

    // --- End Member Variables ---
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}