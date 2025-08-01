#pragma once

// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger

// arena sdk
#include "ArenaApi.h"

class FrameBurstNode : public rclcpp::Node
{
 public:
  FrameBurstNode() : Node("frame_burst_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~FrameBurstNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_burst_srv_;

  // Device parameters
  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;

  size_t width_;
  bool is_passed_width_;

  size_t height_;
  bool is_passed_height_;

  double gain_;
  bool is_passed_gain_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  double frame_rate_;
  bool is_passed_frame_rate_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  // Frame Burst specific parameters
  bool trigger_mode_activated_;
  bool hardware_trigger_;
  bool software_trigger_;
  int burst_frame_count_;
  std::string user_set_;
  bool is_passed_user_set_;

  // QoS parameters
  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  // Image saving parameters
  std::string save_img_folder_;
  bool is_passed_save_img_folder_;

  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_user_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_frame_rate_();
  void set_nodes_frame_burst_trigger_mode_();
  void publish_burst_images_();

  void trigger_frame_burst_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);

  // Image saving functions
  void save_image_to_file_(Arena::IImage* pImage, const sensor_msgs::msg::Image& image_msg);
  void save_as_ppm_(const std::string& filename, Arena::IImage* pImage);
  void save_as_pgm_(const std::string& filename, Arena::IImage* pImage);
  void save_as_raw_(const std::string& filename, Arena::IImage* pImage);
};
