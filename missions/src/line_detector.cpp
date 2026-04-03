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

/*
This node is made using Micka's reply on:
https://stackoverflow.com/questions/35866411/opencv-how-to-detect-lines-of-a-specific-colour

CV bridge docs:
https://docs.ros.org/en/noetic/api/cv_bridge/html/c++/namespacecv__bridge.html#acbf2da402f4d3e505613e95b5a2aed35

Dilation operation:
https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html

Thinning operations:
https://docs.opencv.org/4.x/df/d2d/group__ximgproc.html
*/

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace geometry_msgs::msg;

class LineDetector : public rclcpp::Node
{
public:
LineDetector() : Node("line_detector")
	{		
		// Get down camera image and send back the next target point
		auto image_cb =
		[this](Image::SharedPtr msg) -> void
		{
			RCLCPP_INFO(this->get_logger(), "Received image.");

			// Get rgb image from the message
			rgb_img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

			// Split the image into Hue, Saturation, Value channels and keep the Hue
			cv::cvtColor(rgb_img_ptr->image, hsv_img, cv::COLOR_RGB2HSV);
			cv::split(hsv_img, hsv_channels);
			hue_img = hsv_channels[0];

			// Get color masks
			cv::inRange(hue_img, yellow_hue - yellow_tol, yellow_hue + yellow_tol, yellow_mask);
			cv::inRange(hue_img, green_hue - green_tol, green_hue + green_tol, green_mask);

			// Get additional masks
			saturation_mask = hsv_channels[1] > 50;
			value_mask = hsv_channels[2] > 75;

			// Adapt masks
			yellow_mask = (yellow_mask & saturation_mask) & value_mask;
			green_mask = (green_mask & saturation_mask) & value_mask;

			// Dilate a mask
			cv::dilate(yellow_mask, yellow_mask, dilation_element);
			
			// Use a bitwise and operation to get the overlap
			cv::bitwise_and(yellow_mask, green_mask, overlap_mask);
			
			// Erode the overlap to get a 1 pixel line
			cv::ximgproc::thinning(overlap_mask, overlap_mask, 0);

			// Display the output line
			cv::imshow("OVERLAP", overlap_mask);
			cv::waitKey(1);
		};

		// Variables
		yellow_hue = 30.0; // From hslpicker.com
		yellow_tol = 5.0f;

		green_hue = 57.5f; // From hslpicker.com
		green_tol = 5.0f;

		dilation_size = 16;
		dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size+1), cv::Point(dilation_size, dilation_size));

		/* Publishers and subscribers */
		// auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		target_publisher_ = this->create_publisher<Pose>("/line_detections", 10);
		debug_publisher_ = this->create_publisher<Image>("/debug_img", 10);
		image_subscriber_ = this->create_subscription<Image>("/camera_down/image", 10, image_cb);

	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Publisher and subscriber objects
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	rclcpp::Publisher<Pose>::SharedPtr target_publisher_;
	rclcpp::Publisher<Image>::SharedPtr debug_publisher_;
	rclcpp::Subscription<Image>::SharedPtr image_subscriber_;
	
	// Shared variables
	cv_bridge::CvImagePtr rgb_img_ptr, yellow_img_ptr, green_img_ptr;

	cv::Mat hsv_img, hue_img;
	std::vector<cv::Mat> hsv_channels;

	// Dilation variable
	int dilation_size;
	cv::Mat dilation_element;
	
	cv::Mat yellow_mask, green_mask, saturation_mask, value_mask, overlap_mask;

	// Hue tolerances
	float yellow_hue, yellow_tol, green_hue, green_tol;
};

int main(int argc, char *argv[])
{
	std::cout << "LineDetector node started." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	std::shared_ptr<LineDetector> obj = std::make_shared<LineDetector>();
	rclcpp::spin(obj);
	rclcpp::shutdown();
	return 0;
}
