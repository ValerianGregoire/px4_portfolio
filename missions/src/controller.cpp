/****************************************************************************
 *
 * Copyright 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <std_msgs/msg/header.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <missions/pid_.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace aruco_opencv_msgs::msg;
using namespace geometry_msgs::msg;

class Controller : public rclcpp::Node
{
public:
	Controller() : Node("controller")
	{
		/* Callback functions */
		
		// Gets estimated height from PX4 and converts it from NED to ENU frame
		auto vehicle_odometry_cb =
		[this](VehicleOdometry::UniquePtr msg) -> void
		{
			// Convert from NED to ENU
			z = -(msg->position[2]);
		};

		// Gets the latest aruco detections and updates their positions
		auto aruco_detection_cb =
		[this](ArucoDetection::UniquePtr msg) -> void
		{
			start_aruco_pose = {0.0, 0.0, 0.0}; // id = 0
			end_aruco_pose = {0.0, 0.0, 0.0}; // id = 10
			rover_aruco_pose = {0.0, 0.0, 0.0}; // id = 5
			target_image_pose = {0.0, 0.0, 0.0}; // name = target_image_name
			window_pose = {0.0, 0.0, 0.0}; // closest to image center

			start_aruco_detected = false;
			end_aruco_detected = false;
			rover_aruco_detected = false;
			target_image_detected = false;
			window_detected = false;

			int len = msg->markers.size();
			for (int i=0;i<len;i++)
			{
				MarkerPose marker_ = msg->markers[i];
				uint16_t marker_id = marker_.marker_id;
				std::array<float, 3UL> *pose;
				switch (marker_id)
				{
					case 0:
						pose = &start_aruco_pose;
						start_aruco_detected = true;
						break;

					case 10:
						pose = &end_aruco_pose;
						end_aruco_detected = true;
						break;

					case 5:
						pose = &rover_aruco_pose;
						rover_aruco_detected = true;
						break;
					
					default:
						break;
				}
				(*pose)[0] = marker_.pose.position.x;
				(*pose)[1] = marker_.pose.position.y;
				// RCLCPP_INFO(this->get_logger(), "Received aruco position : id=%d, x=%f, y=%f, z=%f", marker_id, marker_.pose.position.x, marker_.pose.position.y, marker_.pose.position.z);
			}
		};

		// Gets the latest line detection and update the target position
		auto line_detection_cb =
		[this](Pose::UniquePtr msg) -> void
		{
			line_pose[0] = (float)msg->position.y;
			line_pose[1] = (float)msg->position.x;
			line_pose[2] = (float)msg->position.z;
			line_detected = true;
			RCLCPP_INFO(this->get_logger(), "Received line position : x=%f, y=%f", line_pose[0], line_pose[1]);
		};

		/* Publishers and subscribers */
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos, vehicle_odometry_cb);
		aruco_detection_subscriber_ = this->create_subscription<ArucoDetection>("/aruco_detections", qos, aruco_detection_cb);
		line_detection_subscriber_ = this->create_subscription<Pose>("/line_detections", 10, line_detection_cb);


		/* Utilities */
		offboard_control_mode_counter = 0;
		mission_altitude = 3.0;
		descent_speed = -0.2;
		nan = std::nanf("");

		static_pose = {0.0, 0.0, 0.0};
		custom_movement = {-0.08, -0.15, 0.0};


		// Control commands to send to PX4
		auto timer_callback = [this]() -> void
		{
			// Arm the vehicle at the beginning of the mission 
			if (offboard_control_mode_counter == 10)
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm_disarm(true);

				// Begin the mission
				state = TAKEOFF;
			}
			
			// Stop the counter after reaching 11
			if (offboard_control_mode_counter < 11)
			{
				offboard_control_mode_counter++;
			}
			
			// Run the state machine
			behavior();
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	enum MissionState
	{
		TAKEOFF,
		START_ALIGN,
		LINE_FOLLOW,
		IMAGE_ALIGN,
		WINDOW_ALIGN,
		ROVER_ALIGN,
		LAND_ALIGN,
		LANDING,
		LANDED,
		CUSTOM
	};

	enum MissionState state;

	
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Publisher and subscriber objects
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<ArucoDetection>::SharedPtr aruco_detection_subscriber_;
	rclcpp::Subscription<Pose>::SharedPtr line_detection_subscriber_;

	// Utilities
	uint64_t timestamp_ms;
	TrajectorySetpoint setpoint;
	uint64_t offboard_control_mode_counter;

	// Environment cues
	std::array<float, 3UL> static_pose;
	std::array<float, 3UL> custom_movement;
	std::array<float, 3UL> start_aruco_pose;
	std::array<float, 3UL> end_aruco_pose;
	std::array<float, 3UL> rover_aruco_pose;
	std::array<float, 3UL> target_image_pose;
	std::array<float, 3UL> window_pose;
	std::array<float, 3UL> line_pose;
	

	// Flags
	bool start_aruco_detected;
	bool end_aruco_detected;
	bool rover_aruco_detected;
	bool target_image_detected;
	bool window_detected;
	bool line_detected;
	bool aligned;
	
	// Control
	PID pid_x = PID(1.0, 0.0, 0.2);
	PID pid_y = PID(1.0, 0.0, 0.2);
	PID pid_z = PID(1.0, 0.0, 0.2);	// Good control

	// Odometry
	float nan;							// For uncontrolled variables
	float z, x_err, y_err, z_err;		// Coordinates are assumed to be ENU
	float mission_altitude;				// Altitude at which the UAV flies
	float descent_speed;				// Max speed at which the UAV can descend to land


	/* Functions */
	void arm_disarm(bool arm = false);
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(TrajectorySetpoint setpoint);
	void publish_vehicle_command(uint16_t command, float param1, float param2);
	void control(uint64_t timestamp_ms, std::array<float, 3UL> target, float z_tgt = -1.0f, float align_err = 0.05f);
	void behavior();
};

void Controller::arm_disarm(bool arm)
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, arm ? 1.0 : 0.0, 0.0);

	RCLCPP_INFO(this->get_logger(), arm ? "Sent arm command" : "Sent disarm command");
}

void Controller::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.attitude = false;
	msg.acceleration = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void Controller::publish_trajectory_setpoint(TrajectorySetpoint setpoint)
{
	TrajectorySetpoint msg(setpoint);

	// Convert from ENU to NED
	float vx_, vy_;
	vx_ = msg.velocity[0];
	vy_ = msg.velocity[1];

	msg.velocity[0] = vy_;
	msg.velocity[1] = vx_;
	msg.velocity[2] = -msg.velocity[2];
	msg.yawspeed = 0;
	
	msg.position = {nan, nan, nan}; // Make sure to not control position
	msg.yaw = nan;
	msg.acceleration = {nan, nan, nan};

	msg.timestamp = timestamp_ms;
	trajectory_setpoint_publisher_->publish(msg);
}

void Controller::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	msg.timestamp = timestamp_ms;
	vehicle_command_publisher_->publish(msg);
}

void Controller::control(uint64_t timestamp_ms, std::array<float, 3UL> target, float z_tgt, float align_err)
{
	x_err = -target[0];
	y_err = -target[1];
	z_tgt = z_tgt >= 0.0 ? z_tgt : target[2];
	z_err = std::max(z_tgt - z, descent_speed);
	aligned = std::abs(x_err) < align_err && x_err != 0.0 && std::abs(y_err) < align_err && y_err != 0.0;

	setpoint.velocity[1] = pid_x.compute(x_err, timestamp_ms);
	setpoint.velocity[0] = pid_y.compute(y_err, timestamp_ms);
	setpoint.velocity[2] = pid_z.compute(z_err, timestamp_ms);
}

void Controller::behavior()
{
	timestamp_ms = get_clock()->now().nanoseconds() / 1000;
	setpoint = TrajectorySetpoint();

	// State machine
	switch (state)
	{
		case TAKEOFF:
			// RCLCPP_INFO(this->get_logger(), "In takeoff mode.");
			control(timestamp_ms, static_pose, mission_altitude);

			state = ((z > 0.5) && start_aruco_detected) ? START_ALIGN : state; 
			break;		

		case START_ALIGN:
			// RCLCPP_INFO(this->get_logger(), "In start_align mode.");
			control(timestamp_ms, start_aruco_pose, mission_altitude);

			if (aligned)
			{
				state = LINE_FOLLOW; 
			}
			break;	

		case LINE_FOLLOW:
			// RCLCPP_INFO(this->get_logger(), "In line follow mode.");
			control(timestamp_ms, line_pose, mission_altitude);

			if (end_aruco_detected)
			{
				state = LAND_ALIGN;
			}
			break;

		case CUSTOM:
			// RCLCPP_INFO(this->get_logger(), "In custom mode.");
			control(timestamp_ms, custom_movement, mission_altitude);

			if (end_aruco_detected)
			{
				state = LAND_ALIGN;
			}
			break;
		
		case LAND_ALIGN:
			// RCLCPP_INFO(this->get_logger(), "In land_align mode.");
			control(timestamp_ms, end_aruco_pose, mission_altitude);

			if (aligned)
			{
				state = LANDING; 
			}
			break;

		case LANDING:
			// RCLCPP_INFO(this->get_logger(), "In landing mode. z=%f",z);
			control(timestamp_ms, end_aruco_pose, 0.0);
			if (z < 0.25)
			{
				state = LANDED;
			}
			break;
		
		case LANDED:
			// RCLCPP_INFO(this->get_logger(), "In landed mode.");
			arm_disarm(false);
			timer_->cancel();
			break;

		default:
			break;
	}

	// Avoid arena bounds

	// RCLCPP_INFO(this->get_logger(), "Known z: %f", z);
	// Publish commands
	publish_offboard_control_mode();
	publish_trajectory_setpoint(setpoint);
}

int main(int argc, char *argv[])
{
	std::cout << "Controller node started." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	std::shared_ptr<Controller> obj = std::make_shared<Controller>();
	rclcpp::spin(obj);
	rclcpp::shutdown();
	return 0;
}
