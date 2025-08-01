#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>
#include <filesystem> // for directory creation
#include <fstream>    // for file writing
#include <iomanip>    // for formatting

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "FrameBurstNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

void FrameBurstNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    if (this->has_parameter("serial")) {
        int serial_integer;
        this->get_parameter<int>("serial", serial_integer);
        serial_ = std::to_string(serial_integer);
        is_passed_serial_ = true;
    } else {
        serial_ = ""; // Set it to an empty string to indicate it's not passed.
        is_passed_serial_ = false;
    }
    
    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "rgb8");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 1280);
    is_passed_width_ = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 720);
    is_passed_height_ = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    nextParameterToDeclare = "frame_rate";
    frame_rate_ = this->declare_parameter("frame_rate", -1.0);
    is_passed_frame_rate_ = frame_rate_ > 0;

    nextParameterToDeclare = "hardware_trigger";
    hardware_trigger_ = this->declare_parameter("hardware_trigger", true);

    nextParameterToDeclare = "software_trigger";
    software_trigger_ = this->declare_parameter("software_trigger", false);

    nextParameterToDeclare = "burst_frame_count";
    burst_frame_count_ = this->declare_parameter("burst_frame_count", 10);

    nextParameterToDeclare = "user_set";
    user_set_ = this->declare_parameter("user_set", "Default");
    is_passed_user_set_ = user_set_ != "Default";

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/burst_images");

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", true);

    nextParameterToDeclare = "save_img_folder";
    save_img_folder_ = this->declare_parameter("save_img_folder", "");
    is_passed_save_img_folder_ = (save_img_folder_ != "");
    std::cout << "Save image folder: " << save_img_folder_ << "bool: "<< (save_img_folder_ != "") <<std::endl;

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void FrameBurstNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&FrameBurstNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER SERVICE --------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_burst_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_burst",
      std::bind(&FrameBurstNode::trigger_frame_burst_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  rclcpp::SensorDataQoS pub_qos_;
  
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      throw;
    }
  }
  
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

void FrameBurstNode::wait_for_device_timer_callback_()
{
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least one is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discovered.");
    run_();
  }
}

void FrameBurstNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();
  m_pDevice->StartStream();

  if (hardware_trigger_) {
    log_info("Hardware trigger mode: Waiting for trigger signal on Line0...");
    publish_burst_images_();
  } else if (software_trigger_) {
    log_info("Software trigger mode: Use service call to trigger burst");
    // Service will handle software triggers
  } else {
    log_warn("No trigger mode specified. Use hardware_trigger:=true or software_trigger:=true");
  }
}

void FrameBurstNode::publish_burst_images_()
{
  Arena::IImage* pImage = nullptr;
  int images_received = 0;
  
  while (rclcpp::ok()) {
    try {
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      
      // For hardware trigger, use longer timeout to wait for trigger signal
      int timeout_ms = hardware_trigger_ ? 10000 : 2000;
      pImage = m_pDevice->GetImage(timeout_ms);
      
      msg_form_image_(pImage, *p_image_msg);
      m_pub_->publish(std::move(p_image_msg));

      images_received++;
      // log_info(std::string("Burst image ") + std::to_string(images_received) + 
      //     " (Frame ID: " + std::to_string(pImage->GetFrameId()) + 
      //     ", Size: " + std::to_string(pImage->GetWidth()) + "x" + std::to_string(pImage->GetHeight()) +
      //     ", Format: " + std::string(Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "PixelFormat")) +
      //     ") published to " + topic_);
      
      this->m_pDevice->RequeueBuffer(pImage);

      // Check if we received all burst frames
      if (images_received >= burst_frame_count_) {
        log_info(std::string("Completed burst sequence: ") + 
                 std::to_string(images_received) + " images captured");
        images_received = 0;  // Reset counter for next burst
        
        if (software_trigger_) {
          break;  // Exit loop for software trigger mode
        }
      }

    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      
      if (hardware_trigger_) {
        // For hardware trigger, timeout is expected when no trigger signal
        log_debug("Waiting for hardware trigger signal...");
        images_received = 0;  // Reset counter on timeout
      } else {
        log_warn(std::string("Exception occurred while capturing burst images: ") +
                 e.what());
      }
    }
  }
}

void FrameBurstNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // Debug: Log image properties from camera
    auto actual_width = pImage->GetWidth();
    auto actual_height = pImage->GetHeight();
   
    // Check first few bytes of image data
    auto* imageData = static_cast<const uint8_t*>(pImage->GetData());
    // std::string firstBytes = "First 10 bytes: ";
    // for (int i = 0; i < std::min(10, (int)pImage->GetSizeFilled()); i++) {
    //   firstBytes += std::to_string(imageData[i]) + " ";
    // }
    // log_info(firstBytes);
    
    // 1 ) Header
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    // 2 ) Height
    image_msg.height = height_;

    // 3 ) Width
    image_msg.width = width_;

    // 4 ) encoding
    image_msg.encoding = pixelformat_ros_;

    // 5 ) is_big_endian
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    
    // 6 ) step
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    // 7) data
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

    // log_info(std::string("ROS image message: ") + std::to_string(image_msg.width) + "x" + 
    //          std::to_string(image_msg.height) + ", step=" + std::to_string(image_msg.step) + 
    //          ", data_size=" + std::to_string(image_msg.data.size()));

    // Save image to file if folder is specified
    if (is_passed_save_img_folder_) {
      std::cout << "Saving image" << std::endl;
      save_image_to_file_(pImage, image_msg);
    }
    else{
      std::cout << "Image saving folder not specified. Skipping save." << std::endl;
    }

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void FrameBurstNode::trigger_frame_burst_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!software_trigger_) {
    std::string msg =
        "Failed to trigger burst because software trigger mode is not enabled. "
        "Run with --ros-args -p software_trigger:=true";
    log_warn(msg);
    response->message = msg;
    response->success = false;
    return;
  }

  log_info("Software trigger received for frame burst");

  try {
    // Check if trigger is armed
    bool triggerArmed = false;
    auto waitForTriggerCount = 0;
    auto nodemap = m_pDevice->GetNodeMap();
    
    do {
      triggerArmed = Arena::GetNodeValue<bool>(nodemap, "TriggerArmed");
      if (!triggerArmed && (waitForTriggerCount % 100) == 0) {
        log_debug("Waiting for trigger to be armed...");
      }
      waitForTriggerCount++;
    } while (!triggerArmed && waitForTriggerCount < 1000);

    if (!triggerArmed) {
      std::string msg = "Trigger not armed after waiting";
      log_warn(msg);
      response->message = msg;
      response->success = false;
      return;
    }

    log_debug("Trigger is armed; executing software trigger for frame burst");
    Arena::ExecuteNode(nodemap, "TriggerSoftware");

    // Capture burst images
    publish_burst_images_();

    response->message = std::string("Successfully triggered frame burst of ") + 
                       std::to_string(burst_frame_count_) + " images";
    response->success = true;
    log_info(response->message);

  } catch (std::exception& e) {
    auto msg = std::string("Exception occurred during software burst trigger: ") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  } catch (GenICam::GenericException& e) {
    auto msg = std::string("GenICam Exception occurred during software burst trigger: ") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* FrameBurstNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void FrameBurstNode::set_nodes_()
{
  set_nodes_load_user_profile_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_frame_rate_();
  set_nodes_frame_burst_trigger_mode_();
  
  // configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
}

void FrameBurstNode::set_nodes_load_user_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // Load the specified user set (Default, UserSet1, etc.)
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", user_set_.c_str());
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info(std::string("\t") + user_set_ + " profile is loaded");
}

void FrameBurstNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width
  if (is_passed_width_) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height
  if (is_passed_height_) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void FrameBurstNode::set_nodes_gain_()
{
  if (is_passed_gain_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void FrameBurstNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      auto x = std::string("pixelformat is not supported by this camera: ");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelformat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
    }
  }
}

void FrameBurstNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
    log_info(std::string("\tExposure time set to ") + std::to_string(exposure_time_) + " microseconds");
  }
}

void FrameBurstNode::set_nodes_frame_rate_()
{
  if (is_passed_frame_rate_) {
    auto nodemap = m_pDevice->GetNodeMap();
    
    try {
      // Enable frame rate control
      Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", true);
      
      // Set the frame rate
      Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate_);
      
      log_info(std::string("\tFrame rate set to ") + std::to_string(frame_rate_) + " Hz");
      
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("Failed to set frame rate: ") + e.what());
      log_warn("This camera may not support frame rate control or the value may be out of range");
    }
  }
}

void FrameBurstNode::set_nodes_frame_burst_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  
  // Debug: Log current camera state before changes
  log_info("=== DEBUG: Camera state before Frame Burst setup ===");
  try {
    auto current_trigger_mode = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode");
    auto current_pixel_format = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    auto current_width = Arena::GetNodeValue<int64_t>(nodemap, "Width");
    auto current_height = Arena::GetNodeValue<int64_t>(nodemap, "Height");
    
    log_info(std::string("Current TriggerMode: ") + std::string(current_trigger_mode));
    log_info(std::string("Current PixelFormat: ") + std::string(current_pixel_format));
    log_info(std::string("Current Width: ") + std::to_string(current_width));
    log_info(std::string("Current Height: ") + std::to_string(current_height));
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("Failed to read current camera state: ") + e.what());
  }
  
  // Set trigger selector to FrameBurstStart for burst mode
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameBurstStart");
  
  // Re-enable trigger mode after setting selector (some cameras reset it)
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
  
  // Set the number of frames in the burst
  Arena::SetNodeValue<int64_t>(nodemap, "AcquisitionBurstFrameCount", burst_frame_count_);
  
  if (hardware_trigger_) {
    // Hardware trigger setup for Line0
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Line0");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerActivation", "RisingEdge");
    
    // Line0 configuration
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "LineSelector", "Line0");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "LineMode", "Input");
    
    log_info(std::string("\tHardware trigger mode activated on Line0 for frame burst (") + 
             std::to_string(burst_frame_count_) + " frames)");
    log_info("\tTrigger activation: Rising Edge");
    
  } else if (software_trigger_) {
    // Software trigger setup
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Software");
    
    log_info(std::string("\tSoftware trigger mode activated for frame burst (") + 
             std::to_string(burst_frame_count_) + " frames)");
    log_info(std::string("\tTo trigger burst, call service: ros2 service call /") + 
             this->get_name() + "/trigger_burst std_srvs/srv/Trigger");
  }
  
  // Debug: Log final camera state after changes
  log_info("=== DEBUG: Camera state after Frame Burst setup ===");
  try {
    auto final_trigger_mode = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode");
    auto final_trigger_selector = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector");
    auto final_trigger_source = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource");
    auto final_burst_count = Arena::GetNodeValue<int64_t>(nodemap, "AcquisitionBurstFrameCount");
    
    log_info(std::string("Final TriggerMode: ") + std::string(final_trigger_mode));
    log_info(std::string("Final TriggerSelector: ") + std::string(final_trigger_selector));
    log_info(std::string("Final TriggerSource: ") + std::string(final_trigger_source));
    log_info(std::string("Final AcquisitionBurstFrameCount: ") + std::to_string(final_burst_count));
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("Failed to read final camera state: ") + e.what());
  }
}

void FrameBurstNode::save_image_to_file_(Arena::IImage* pImage, const sensor_msgs::msg::Image& image_msg)
{
  try {
    // Create directory if it doesn't exist
    std::filesystem::create_directories(save_img_folder_);
    
    // Generate timestamp-based filename with millisecond precision
    uint64_t timestamp_ns = pImage->GetTimestampNs();
    double timestamp_seconds = static_cast<double>(timestamp_ns) / 1000000000.0;
    
    std::ostringstream filename;
    filename << save_img_folder_ << "/" << std::fixed << std::setprecision(3) << timestamp_seconds;

    std::cout << "Saving image to: " << filename.str() << std::endl;
    
    // Save based on pixel format
    if (pixelformat_ros_ == "rgb8" || pixelformat_ros_ == "bgr8") {
      save_as_ppm_(filename.str() + ".ppm", pImage);
    } else if (pixelformat_ros_ == "mono8") {
      save_as_pgm_(filename.str() + ".pgm", pImage);
    } else {
      save_as_raw_(filename.str() + ".raw", pImage);
    }
    
    log_info(std::string("Image saved: ") + filename.str());
    
  } catch (const std::exception& e) {
    log_warn(std::string("Failed to save image: ") + e.what());
  }
}

void FrameBurstNode::save_as_ppm_(const std::string& filename, Arena::IImage* pImage)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file for writing: " + filename);
  }
  
  // PPM header
  file << "P6\n" << pImage->GetWidth() << " " << pImage->GetHeight() << "\n255\n";
  
  // Write image data
  const uint8_t* imageData = static_cast<const uint8_t*>(pImage->GetData());
  size_t dataSize = pImage->GetWidth() * pImage->GetHeight() * 3; // RGB = 3 bytes per pixel
  
  if (pixelformat_ros_ == "bgr8") {
    // Convert BGR to RGB for PPM format
    for (size_t i = 0; i < dataSize; i += 3) {
      file.write(reinterpret_cast<const char*>(&imageData[i + 2]), 1); // R
      file.write(reinterpret_cast<const char*>(&imageData[i + 1]), 1); // G
      file.write(reinterpret_cast<const char*>(&imageData[i + 0]), 1); // B
    }
  } else {
    // RGB format - write directly
    file.write(reinterpret_cast<const char*>(imageData), dataSize);
  }
  
  file.close();
}

void FrameBurstNode::save_as_pgm_(const std::string& filename, Arena::IImage* pImage)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file for writing: " + filename);
  }
  
  // PGM header
  file << "P5\n" << pImage->GetWidth() << " " << pImage->GetHeight() << "\n255\n";
  
  // Write image data
  const uint8_t* imageData = static_cast<const uint8_t*>(pImage->GetData());
  size_t dataSize = pImage->GetWidth() * pImage->GetHeight(); // Mono = 1 byte per pixel
  file.write(reinterpret_cast<const char*>(imageData), dataSize);
  
  file.close();
}

void FrameBurstNode::save_as_raw_(const std::string& filename, Arena::IImage* pImage)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file for writing: " + filename);
  }
  
  // Write raw image data
  const uint8_t* imageData = static_cast<const uint8_t*>(pImage->GetData());
  size_t dataSize = pImage->GetSizeFilled();
  file.write(reinterpret_cast<const char*>(imageData), dataSize);
  
  file.close();
  
  // Create accompanying info file
  std::ofstream infoFile(filename + ".info");
  if (infoFile.is_open()) {
    infoFile << "Width: " << pImage->GetWidth() << "\n";
    infoFile << "Height: " << pImage->GetHeight() << "\n";
    infoFile << "PixelFormat: " << pixelformat_ros_ << "\n";
    infoFile << "BitsPerPixel: " << pImage->GetBitsPerPixel() << "\n";
    infoFile << "DataSize: " << dataSize << "\n";
    infoFile.close();
  }
}
