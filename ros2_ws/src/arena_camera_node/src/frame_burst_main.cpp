#include "FrameBurstNode.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<FrameBurstNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
