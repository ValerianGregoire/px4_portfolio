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

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <missions/pid_.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Controller : public rclcpp::Node
{
public:
	Controller() : Node("controller")
	{
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		auto vehicle_odometry_cb =
			[this](VehicleOdometry::UniquePtr msg) -> void
		{
			x = msg->position[0];
			y = msg->position[1];
			z = msg->position[2];
		};

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos, vehicle_odometry_cb);

		offboard_control_mode_counter = 0;

		auto timer_callback = [this]() -> void
		{
			if (offboard_control_mode_counter == 10)
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm_disarm(true);
			}

			if (offboard_control_mode_counter >= 250)
			{
				if (z < -0.2)
				{
					land();
				}
				else
				{
					arm_disarm();
				}
			}
			else if (z > virtual_ground)
			{
				takeoff();
			}
			else
			{
				mission();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode(control_mode);
			publish_trajectory_setpoint(control_mode, setpoint);

			// stop the counter after reaching 1200 (120s)
			if (offboard_control_mode_counter < 1200)
			{
				offboard_control_mode_counter++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

	TrajectorySetpoint setpoint;
	OffboardControlMode control_mode;
	uint64_t offboard_control_mode_counter;

	// Values
	float nan = std::nanf("");
	float x, y, z;
	float virtual_ground = -15.0;

	void arm_disarm(bool arm = false);
	void publish_offboard_control_mode(OffboardControlMode mode);
	void publish_trajectory_setpoint(OffboardControlMode mode, TrajectorySetpoint setpoint);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void takeoff();
	void land();
	void mission();
};

void Controller::arm_disarm(bool arm)
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, arm ? 1.0 : 0.0);

	RCLCPP_INFO(this->get_logger(), arm ? "Sent arm command" : "Sent disarm command");
}

void Controller::publish_offboard_control_mode(OffboardControlMode mode)
{
	OffboardControlMode msg(mode);
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void Controller::publish_trajectory_setpoint(OffboardControlMode mode, TrajectorySetpoint setpoint)
{
	TrajectorySetpoint msg(setpoint);

	if (!mode.position)
	{
		msg.position = {nan, nan, nan};
	}
	else if (!mode.velocity)
	{
		msg.velocity = {nan, nan, nan};
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void Controller::takeoff()
{
	setpoint = TrajectorySetpoint();
	setpoint.position = {nan, nan, virtual_ground - 2.0f};
	setpoint.yaw = nan;
	control_mode.position = true;
}

void Controller::land()
{
	setpoint = TrajectorySetpoint();
	setpoint.position = {nan, nan, 0};
	if (z < -1.5)
	{
		setpoint.velocity = {nan, nan, -z};
	}
	else
	{
		setpoint.velocity = {nan, nan, 0.2};
	}
	setpoint.yaw = nan;
	control_mode.position = true;
	control_mode.velocity = true;
}

void Controller::mission()
{
	setpoint = TrajectorySetpoint();
	setpoint.velocity = {0, 0.2, 0};
	setpoint.yaw = nan;
	setpoint.yawspeed = 0.7;
	control_mode.position = false;
	control_mode.velocity = true;
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
