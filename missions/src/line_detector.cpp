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

Houghlines P:
https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb

Contours:
https://docs.opencv.org/4.x/df/d0d/tutorial_find_contours.html

*/


#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <Eigen/Dense>

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
		cv::cvtColor(rgb_img_ptr->image, bgr_img, cv::COLOR_RGB2BGR);
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
		cv::dilate(yellow_mask, fat_yellow_mask, dilation_element);
		
		// Use a bitwise and operation to get the overlap
		cv::bitwise_and(fat_yellow_mask, green_mask, overlap_mask);
		
		// Use thinning to get 1 pixel lines
		cv::ximgproc::thinning(overlap_mask, thin_overlap_mask, 0);
		cv::ximgproc::thinning(yellow_mask, thin_yellow_mask, 0);

		// Init crop mask if needed
		if (crop_mask.empty()) {
			crop_mask = cv::Mat::zeros(overlap_mask.size(), CV_8UC1);
			crop_mask(cv::Rect(crop_size, crop_size, overlap_mask.cols - 2*crop_size, overlap_mask.rows - 2*crop_size)).setTo(255);
		}

		// Slightly crop the thin masks to reduce contours size
		cv::bitwise_and(thin_overlap_mask, crop_mask, thin_overlap_mask);
		cv::bitwise_and(thin_yellow_mask, crop_mask, thin_yellow_mask);

		// Get Contours
		cv::findContours(thin_overlap_mask, overlap_contours, overlap_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(thin_yellow_mask, yellow_contours, yellow_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		// Compute closest points pairs from yellow/overlap contours
		contour_pairs.clear();
		for (size_t i = 0; i < overlap_contours.size(); i++)
		{
			for (size_t j = 0; j < overlap_contours[i].size(); j++)
			{
				cv::Point *p1 = &overlap_contours[i][j];
				Eigen::Vector2d p1_v = Eigen::Vector2d(p1->x, p1->y);
				std::vector<cv::Point> pair;
				pair.push_back(*p1);
				pair.push_back(cv::Point(0, 0));
				double distance = 999999.9;
				
				for (size_t k = 0; k < yellow_contours.size(); k++)
				{
					for (size_t l = 0; l < yellow_contours[k].size(); l++)
					{
						cv::Point *p2 = &yellow_contours[k][l];
						Eigen::Vector2d p2_v = Eigen::Vector2d(p2->x, p2->y);
						double distance_temp = (p2_v - p1_v).norm();
						if (distance_temp < distance)
						{
							distance = distance_temp;
							pair[1] = *p2;
						}
					}
				}
				contour_pairs.push_back(pair);
			}
		}

		// Get the index of point closest to the center of the image
		Eigen::Vector2d image_center = Eigen::Vector2d(overlap_mask.cols / 2, overlap_mask.rows / 2);
		int closest_index = -1;
		double closest_distance = 999999.9;
		for (size_t i = 0; i < contour_pairs.size(); i++)
		{
			cv::Point *p1 = &contour_pairs[i][0];
			Eigen::Vector2d p1_v = Eigen::Vector2d(p1->x, p1->y);
			double distance = (image_center - p1_v).norm();
			if (distance < closest_distance)
			{
				closest_distance = distance;
				closest_index = i;
			}
		}

		// Compute target destination and orientation from the point
		if (closest_index != -1)
		{
			cv::Point p1 = contour_pairs[closest_index][0];
			cv::Point p2 = contour_pairs[closest_index][1];
			Eigen::Vector2d pair_center = Eigen::Vector2d((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
			Eigen::Vector2d pair_direction = Eigen::Vector2d(p2.x - p1.x, p2.y - p1.y).normalized();
			double angle = atan2(pair_direction.y(), pair_direction.x());
			
			// Compute the x, y coordinates of the target point in the camera frame
			double target_distance = 0.0;
			target_x = pair_center.x() + target_distance * cos(angle);
			target_y = pair_center.y() + target_distance * sin(angle);

			// Go from pixel coordinates to normalized coordinates between -0.5 and 0.5, with (0, 0) being the center of the image
			target_x = (target_x - overlap_mask.cols / 2) / overlap_mask.cols;
			target_y = -(target_y - overlap_mask.rows / 2) / overlap_mask.rows;

			// Publish the target point as a Pose message
			Pose target_msg;
			target_msg.position.x = target_x;
			target_msg.position.y = target_y;
			target_msg.orientation.z = sin(angle / 2);
			target_msg.orientation.w = cos(angle / 2);
			target_publisher_->publish(target_msg);
			RCLCPP_INFO(this->get_logger(), "Published target point: (%.2f, %.2f) with angle %.2f degrees", target_x, target_y, angle * 180 / M_PI);
		}

		// Output image
		output_image = cv::Mat::zeros(bgr_img.size(), CV_8UC3);
		bgr_img.copyTo(output_image, green_mask);
		bgr_img.copyTo(output_image, yellow_mask);
		bgr_img.copyTo(output_image, overlap_mask);

		for(size_t i = 0; i < yellow_contours.size(); i++) {
			cv::drawContours(output_image, yellow_contours, (int)i, cv::Scalar(0, 0, 255), 2);
		}
		for(size_t i = 0; i < overlap_contours.size(); i++) {
			cv::drawContours(output_image, overlap_contours, (int)i, cv::Scalar(0, 0, 255), 2);
		}

		// Add the target point to the output image using target_x and target_y
		if (closest_index != -1)
		{
			cv::Point p1 = contour_pairs[closest_index][0];
			// cv::Point p2 = contour_pairs[closest_index][1];
			cv::Point target_point = cv::Point((int)(target_x * (overlap_mask.cols / 2) + overlap_mask.cols / 2), (int)(target_y * (overlap_mask.rows / 2) + overlap_mask.rows / 2));
			cv::circle(output_image, p1, 5, cv::Scalar(255, 0, 0), -1);
			// cv::circle(output_image, p2, 5, cv::Scalar(255, 0, 0), -1);
			cv::circle(output_image, target_point, 5, cv::Scalar(255, 0, 0), -1);
			// cv::line(output_image, p1, p2, cv::Scalar(255, 0, 0), 2);
			cv::arrowedLine(output_image, p1, target_point, cv::Scalar(255, 0, 0), 2);
		}
		cv::imshow("OUTPUT", output_image);
		cv::waitKey(1);
	};

	// Variables
	yellow_hue = 30.0; // From hslpicker.com
	yellow_tol = 5.0f;

	green_hue = 57.5f; // From hslpicker.com
	green_tol = 5.0f;

	rng = cv::RNG(12345);
	crop_size = 2;

	dilation_size = 3;
	dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size+1), cv::Point(dilation_size, dilation_size));

	/* Publishers and subscribers */
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

	cv::Mat hsv_img, hue_img, bgr_img;
	std::vector<cv::Mat> hsv_channels;

	// Dilation variable
	int dilation_size;
	cv::Mat dilation_element;

	// Process masks
	cv::Mat yellow_mask, fat_yellow_mask, thin_yellow_mask;
	cv::Mat green_mask;
	cv::Mat overlap_mask, thin_overlap_mask;
	cv::Mat saturation_mask, value_mask;
	cv::Mat output_image, output_mask;

	// Contours
	std::vector<std::vector<cv::Point>> overlap_contours;
    std::vector<cv::Vec4i> overlap_hierarchy;
	std::vector<std::vector<cv::Point>> yellow_contours;
    std::vector<cv::Vec4i> yellow_hierarchy;
	cv::RNG rng;

	// Points pairs
	std::vector<std::vector<cv::Point>> contour_pairs;

	// Image crop
	cv::Mat crop_mask;
	int crop_size;

	double target_x;
	double target_y;

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
