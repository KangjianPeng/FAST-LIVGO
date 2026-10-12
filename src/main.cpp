#include "LIVMapper.h"
#include "optimization.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("laserMapping");
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh);
  optimization optimizer(nh);
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}
