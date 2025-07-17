#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <cmath>

class RotateService {
    private:
        ros::NodeHandle node_handle_;
        ros::ServiceServer rotate_service_;
        ros::Publisher velocity_publisher_;
        ros::Subscriber odom_subscriber_;
        float current_orientation_z_ = 0.0;
        float current_orientation_w_ = 0.0;
        geometry_msgs::Twist twist_command_;

    public:
        RotateService() {
            rotate_service_ = node_handle_.advertiseService("/rotate_robot", &RotateService::processRotationRequest, this);

            odom_subscriber_ = node_handle_.subscribe("/odom", 1000, &RotateService::odomCallback, this);

            velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            ROS_INFO("Service server /rotate_robot is ready to use.");
        }

        bool processRotationRequest(my_rb1_ros::Rotate::Request &request, my_rb1_ros::Rotate::Response &response) {
            ROS_INFO("Received request at /rotate_robot.");
            ROS_INFO("Rotating %d degrees.", request.degrees);

            try {
                int initial_angle = getYawInDegrees(current_orientation_z_, current_orientation_w_);
                int remaining_rotation = std::abs(request.degrees);

                // twist_command_.angular.z = (request.degrees > 0) ? 0.4 : -0.4; 
                twist_command_.angular.z = (request.degrees > 0) ? -0.4 : 0.4; 
                velocity_publisher_.publish(twist_command_);

                while (remaining_rotation > 0) {
                    int updated_angle = getYawInDegrees(current_orientation_z_, current_orientation_w_);
                    if (updated_angle != initial_angle) {
                        remaining_rotation -= 1;
                        initial_angle = updated_angle;
                        ROS_DEBUG("Current angle: %d", initial_angle);
                    }
                    ros::spinOnce();
                }

                twist_command_.angular.z = 0.0;
                velocity_publisher_.publish(twist_command_);

                ROS_INFO("Rotation completed successfully.");
                response.result = "Rotation completed successfully.";
            } catch (...) {
                response.result = "Rotation failed due to an internal error.";
            }
            return true;
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
            current_orientation_w_ = odom_msg->pose.pose.orientation.w;
            current_orientation_z_ = odom_msg->pose.pose.orientation.z;
        }

        int getYawInDegrees(double z, double w) {
            double angle_rad = 2 * std::atan2(z, w);
            return static_cast<int>(angle_rad * (180.0 / M_PI));
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_service_server");
    RotateService rotate_service_instance;
    ros::spin();
    return 0;
}
