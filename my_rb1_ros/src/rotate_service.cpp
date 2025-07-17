#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <cmath>

class RotateService {
private:
  ros::NodeHandle nh_;
  ros::ServiceServer serviceServer_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double current_yaw_;
  bool odom_received_;

public:
  RotateService() {
    current_yaw_ = 0.0;
    odom_received_ = false;

    // Initialize the service server
    serviceServer_ = nh_.advertiseService("/rotate_robot",
                                          &RotateService::handleRequest, this);
    // Initialize the subscriber
    sub_ = nh_.subscribe("/odom", 1000, &RotateService::odomCallback, this);

    // Initialize the publisher
    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ROS_INFO("Service Ready");
  }

  bool handleRequest(my_rb1_ros::Rotate::Request &req,
                     my_rb1_ros::Rotate::Response &res) {

    ROS_INFO("Service Requested");

    // Wait for odometry data
    while (!odom_received_) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }

    try {
      // Get initial angle
      double start_yaw = current_yaw_;
      double target_rotation = req.degrees * M_PI / 180.0; // Convert to radians
      double target_yaw = start_yaw + target_rotation;

      // Normalize target yaw to [-pi, pi]
      target_yaw = normalizeAngle(target_yaw);

      // Set rotation direction and speed
      geometry_msgs::Twist cmd;
      double angular_speed = 0.5; // rad/s
      cmd.angular.z = (req.degrees > 0) ? angular_speed : -angular_speed;

      ros::Rate rate(10); // 10 Hz

      while (ros::ok()) {
        double angle_diff = normalizeAngle(target_yaw - current_yaw_);

        // Check if within tolerance
        if (std::abs(angle_diff) < 0.035) {
          break;
        }
        // Adjust speed
        if (std::abs(angle_diff) < 0.2) {
          cmd.angular.z = (angle_diff > 0) ? 0.2 : -0.2;
        } else {
          cmd.angular.z = (angle_diff > 0) ? angular_speed : -angular_speed;
        }

        pub_.publish(cmd);
        ros::spinOnce();
        rate.sleep();
      }

      // Stop the robot
      cmd.angular.z = 0.0;
      pub_.publish(cmd);

      ROS_INFO("Service Completed");
      res.result = "Rotation completed successfully";

    } catch (...) {
      // Stop the robot in case of error
      geometry_msgs::Twist stop_cmd;
      stop_cmd.angular.z = 0.0;
      pub_.publish(stop_cmd);

      res.result = "Rotation failed";
      return true;
    }

    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Extract yaw angle from quaternion
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // Convert quaternion to yaw angle
    current_yaw_ = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    odom_received_ = true;
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_server");
  RotateService server;

  ros::spin();

  return 0;
}