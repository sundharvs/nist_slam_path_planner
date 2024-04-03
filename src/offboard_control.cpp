#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


// #include <tf2_msgs/msg/tf_message.h>
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <stdint.h>

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;



class WaypointNav : public rclcpp::Node
{
    public:
        WaypointNav() : Node("waypoint_nav")
        {
            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
			odometry_publisher_ = this->create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
			path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("/path",10,std::bind(&WaypointNav::path_callback, this, _1));
			lidar_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/nav/odom_incremental",
				rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
				std::bind(&WaypointNav::lidar_pose_callback, this, _1)
			);

            offboard_setpoint_counter_ = 0;
            auto timer_callback = [this]() -> void {
                if (offboard_setpoint_counter_ == 10) {
                    // Change to Offboard mode after 10 setpoints
                    // this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    // Arm the vehicle
                    // this->arm();
                }

                // offboard_control_mode needs to be paired with trajectory_setpoint
                publish_offboard_control_mode();
                publish_trajectory_setpoint();

                // stop the counter after reaching 11
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
		    };
		    timer_ = this->create_wall_timer(100ms, timer_callback);
        }

        void arm();
        void disarm();
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		rclcpp::Publisher<VehicleOdometry>::SharedPtr odometry_publisher_;
		
		
		rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_subscription_;

		// std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		// std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

        uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

        void publish_offboard_control_mode();
        void publish_trajectory_setpoint();
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
		void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
		void lidar_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

		bool waypoint_received = true;
		bool path_received = false;
		std::array<float,3> waypoint_pos = {0,0,-0.6};
		float waypoint_yaw;
		std::vector<geometry_msgs::msg::PoseStamped> plan_path;
		int path_progress = 0;
};

void WaypointNav::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void WaypointNav::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 */
void WaypointNav::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void WaypointNav::publish_trajectory_setpoint()
{
	if(waypoint_received){
		TrajectorySetpoint msg{};
		msg.position = waypoint_pos;
		msg.yaw = waypoint_yaw; // [-PI:PI]
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void WaypointNav::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}


void WaypointNav::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
	plan_path = msg->poses;
	path_progress = 0;
	path_received = true;
	waypoint_yaw = atan2(plan_path.back().pose.position.x - plan_path[0].pose.position.x, plan_path.back().pose.position.y - plan_path[0].pose.position.y);
}

void WaypointNav::lidar_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	// geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("lidar_link", "odom",tf2::TimePointZero);

	VehicleOdometry odom_msg;

	odom_msg.pose_frame = 2; // 1 for NED earth-fixed frame, 2 for FRD world-fixed frame

	// odom_msg.position = {t.transform.translation.y, t.transform.translation.x, -(t.transform.translation.z)};
	odom_msg.position = {msg->pose.pose.position.y, msg->pose.pose.position.x, -(msg->pose.pose.position.z)};

	Eigen::Quaternionf q2(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	// tf2::Matrix3x3 m(q2);

	Eigen::Matrix3f m1;
	m1(0,1) = 1;
	m1(1,0) = 1;
	m1(2,2) = -1;

	Eigen::Matrix3f m3;
	m3(0,0) = 1;
	m3(1,1) = -1;
	m3(2,2) = -1;

	Eigen::Quaternionf q1(m1);
	Eigen::Quaternionf q3(m3);
	
	Eigen::Quaternionf q = q1*q2*q3;
	
	// double roll, pitch, yaw;
	// m.getRPY(roll, pitch, yaw);
	// q.setRPY(roll-0.34906585, pitch, yaw); // UNCOMMENT THIS FOR REAL LIFE

	// odom_msg.q = {t.transform.rotation.w, -(t.transform.rotation.z;), t.transform.rotation.x, -(t.transform.rotation.y)};
	odom_msg.q = {q.x(),q.y(),q.z(),q.w()};

	odom_msg.velocity_frame = 1;
	odom_msg.velocity = {msg->twist.twist.linear.y, msg->twist.twist.linear.x, -(msg->twist.twist.linear.z)};
	odom_msg.angular_velocity = {msg->twist.twist.angular.y, msg->twist.twist.angular.x, -(msg->twist.twist.angular.z)}; // in body-fixed frame (rad/s)

	odom_msg.position_variance = {0.0, 0.0, 0.0};
	odom_msg.orientation_variance = {0.0, 0.0, 0.0};
	odom_msg.velocity_variance = {0.0, 0.0, 0.0};

	odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	odometry_publisher_->publish(odom_msg);

	if(path_received){
		Eigen::Vector3d current_pose(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
		Eigen::Vector3d next_setpoint(plan_path[path_progress].pose.position.x, plan_path[path_progress].pose.position.y, plan_path[path_progress].pose.position.z);

		if((next_setpoint-current_pose).norm() < 0.2 && path_progress + 1 < plan_path.size()){
			waypoint_pos = {plan_path[path_progress+1].pose.position.y, plan_path[path_progress+1].pose.position.x, -(plan_path[path_progress+1].pose.position.z)};
			path_progress++;
			waypoint_received = true;
		}
	}
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WaypointNav>());

	rclcpp::shutdown();
	return 0;
}
