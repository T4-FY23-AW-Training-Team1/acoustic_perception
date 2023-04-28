// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <string>
#include <libusb-1.0/libusb.h>

#include "rclcpp/rclcpp.hpp"
#include "acoustics_msgs/msg/sound_source_direction.hpp"
#include "visualization_msgs/msg/marker.h"

#define VENDOR_ID 0x2886  // ReSpeakerのVendor ID
#define PRODUCT_ID 0x0018  // ReSpeakerのProduct ID

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RespeakerPublisher : public rclcpp::Node
{
  public:
    RespeakerPublisher()
    : Node("respeaker_driver")
    {
      direction_publisher_ = this->create_publisher<acoustics_msgs::msg::SoundSourceDirection>("sound_source_direction", 10);
      arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("sound_source_direction/arrow", 10);
      timer_ = this->create_wall_timer(
        20ms, std::bind(&RespeakerPublisher::timer_callback, this));

      libusb_init(NULL);
      dev_handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
      if (dev_handle == NULL) {
        RCLCPP_INFO(this->get_logger(), "Cannot open audio device");
      }
      else{
        retDir = getDirection(dev_handle); 
        RCLCPP_INFO(this->get_logger(), "Audio device opened, angle: %d", retDir);
      }
    }

  private:
    void timer_callback(){
      auto direction_message = acoustics_msgs::msg::SoundSourceDirection();
      auto arrow_message = visualization_msgs::msg::Marker();
      direction_message.header.stamp = this->now();
      direction_message.header.frame_id = "sensor_kit_base_link";
      arrow_message.header = direction_message.header;
      arrow_message.ns = "direction_arrow";
      arrow_message.id = 0;
      arrow_message.type = visualization_msgs::msg::Marker::ARROW;
      arrow_message.action = visualization_msgs::msg::Marker::ADD;

      // Get direction and activity from microphone array
      retDir = getDirection(dev_handle);
      retVAD = getVAD(dev_handle);
      if(retDir >= 0){
        float temprad = float(retDir)*M_PI/180;
        direction_message.unit_direction_x = cos(temprad);
        direction_message.unit_direction_y = sin(temprad);
      }else{
        direction_message.unit_direction_x = 0;
        direction_message.unit_direction_y = 0;
        RCLCPP_INFO(this->get_logger(), "Cannot get sound source direction");
      }

      if(retVAD >=0){
        direction_message.activity = retVAD;
      }else{
        direction_message.activity = -1;
        RCLCPP_INFO(this->get_logger(), "Cannot get voice activity");
      }

      // Illustrate the estimated direction
      int r = 3; // ratio of the arrow size

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = 0.0;
      start_point.y = 0.0;
      start_point.z = 0.0;
      end_point.x = direction_message.unit_direction_x * r;
      end_point.y = direction_message.unit_direction_y * r;
      end_point.z = 0.0 * r;
      arrow_message.points.push_back(start_point);
      arrow_message.points.push_back(end_point);

      // Set arrow scale
      arrow_message.scale.x = 0.1*r; // shaft diameter
      arrow_message.scale.y = 0.2*r; // head diameter
      arrow_message.scale.z = 0.5*r; // head length

      // Set arrow color
      if(retVAD == 1){
        arrow_message.color.r = 1.0;
        arrow_message.color.g = 0.0;
        arrow_message.color.b = 0.0;
        arrow_message.color.a = 1.0;
      }else{
        arrow_message.color.r = 0.0;
        arrow_message.color.g = 1.0;
        arrow_message.color.b = 0.0;
        arrow_message.color.a = 0.5;
      }

      RCLCPP_INFO(this->get_logger(), "Publishing SoundSourceDirection: '%.2f, %.2f, %d'", direction_message.unit_direction_x, direction_message.unit_direction_y, direction_message.activity);
      direction_publisher_->publish(direction_message);
      arrow_publisher_->publish(arrow_message);
    }

    int getDirection(libusb_device_handle* handle){
      int ret;
      //Paramters
      uint8_t request_type = 0x80 | (2<<5) | 0;
      uint8_t request = 0x00;
      uint16_t value = 0x80 | 0x00 | 0x40;
      uint16_t index = 21;
      unsigned char buffer[8];
      int length = 8;
      int timeout = 10000;
      //std::cout << "buffer is: " << typeid(buffer).name() << std::endl;
      ret = libusb_control_transfer(handle, request_type, request, value, index, buffer, length, timeout);
      if(ret >= 0){
          ret = int((buffer[1]<<8) | buffer[0]);
      }

      return ret;
    }

    int getVAD(libusb_device_handle* handle){
        int ret;
        //Paramters
        uint8_t request_type = 0x80 | (2<<5) | 0;
        uint8_t request = 0x00;
        uint16_t value = 0x80 | 32 | 0x40;
        uint16_t index = 19;
        unsigned char buffer[8];
        int length = 8;
        int timeout = 10000;
        ret = libusb_control_transfer(handle, request_type, request, value, index, buffer, length, timeout);
        if(ret >= 0){
            ret = int(buffer[0]);
        }

        return ret;
    }

    int getSpeechDetection(libusb_device_handle* handle){
        int ret;
        //Paramters
        uint8_t request_type = 0x80 | (2<<5) | 0;
        uint8_t request = 0x00;
        uint16_t value = 0x80 | 22 | 0x40;
        uint16_t index = 19;
        unsigned char buffer[8];
        int length = 8;
        int timeout = 10000;
        ret = libusb_control_transfer(handle, request_type, request, value, index, buffer, length, timeout);
        if(ret >= 0){
            ret = int(buffer[0]);
        }

        return ret;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<acoustics_msgs::msg::SoundSourceDirection>::SharedPtr direction_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_publisher_;
    int retDir, retVAD, retSpch;
    libusb_device_handle *dev_handle;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RespeakerPublisher>());
  rclcpp::shutdown();
  return 0;
}
