#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>
#include "rclcpp/rclcpp.hpp"

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
/// 相比ros，传入参数上多了一个node参数，调用要注意
bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  
  // 获取相机模型
  std::string cam_model = node->get_parameter(ns + "/cam_model").get_value<std::string>();

  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(node->get_parameter(ns + "/cam_calib_file").get_value<std::string>());
  }
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        node->get_parameter(ns + "/scale", 1.0).get_value<double>(),
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        node->get_parameter(ns + "/cam_d0", 0.0).get_value<double>(),
        node->get_parameter(ns + "/cam_d1", 0.0).get_value<double>(),
        node->get_parameter(ns + "/cam_d2", 0.0).get_value<double>(),
        node->get_parameter(ns + "/cam_d3", 0.0).get_value<double>());
  }
  else if(cam_model == "EquidistantCamera")
  {
    cam = new vk::EquidistantCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        node->get_parameter(ns + "/scale", 1.0).get_value<double>(),
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        node->get_parameter(ns + "/k1", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k2", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k3", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k4", 0.0).get_value<double>());
  }
  else if(cam_model == "PolynomialCamera")
  {
    cam = new vk::PolynomialCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        node->get_parameter(ns + "/cam_skew").get_value<double>(),
        node->get_parameter(ns + "/k2", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k3", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k4", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k5", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k6", 0.0).get_value<double>(),
        node->get_parameter(ns + "/k7", 0.0).get_value<double>());
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        node->get_parameter(ns + "/cam_d0").get_value<double>());
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool res = true;

  std::string cam_model = node->get_parameter(ns + "/cam_model").get_value<std::string>();
  int cam_num = node->get_parameter(ns + "/cam_num").get_value<int>();
  
  for (int i = 0; i < cam_num; i++)
  {
    std::string cam_ns = ns + "/cam_" + std::to_string(i);
    std::string cam_model = node->get_parameter(cam_ns + "/cam_model").get_value<std::string>();
    
    if(cam_model == "FishPoly")
    {
      cam_list.push_back(new vk::PolynomialCamera(
        node->get_parameter(cam_ns + "/image_width").get_value<int>(),
        node->get_parameter(cam_ns + "/image_height").get_value<int>(),
        node->get_parameter(cam_ns + "/A11").get_value<double>(),  // cam_fx
        node->get_parameter(cam_ns + "/A22").get_value<double>(),  // cam_fy
        node->get_parameter(cam_ns + "/u0").get_value<double>(),  // cam_cx
        node->get_parameter(cam_ns + "/v0").get_value<double>(),  // cam_cy
        node->get_parameter(cam_ns + "/A12").get_value<double>(), // cam_skew
        node->get_parameter(cam_ns + "/k2", 0.0).get_value<double>(),
        node->get_parameter(cam_ns + "/k3", 0.0).get_value<double>(),
        node->get_parameter(cam_ns + "/k4", 0.0).get_value<double>(),
        node->get_parameter(cam_ns + "/k5", 0.0).get_value<double>(),
        node->get_parameter(cam_ns + "/k6", 0.0).get_value<double>(),
        node->get_parameter(cam_ns + "/k7", 0.0).get_value<double>()));
    }
    else if(cam_model == "Pinhole")
    {
      cam_list.push_back(new vk::PinholeCamera(
          node->get_parameter(ns + "/cam_width").get_value<int>(),
          node->get_parameter(ns + "/cam_height").get_value<int>(),
          node->get_parameter(ns + "/scale", 1.0).get_value<double>(),
          node->get_parameter(ns + "/cam_fx").get_value<double>(),
          node->get_parameter(ns + "/cam_fy").get_value<double>(),
          node->get_parameter(ns + "/cam_cx").get_value<double>(),
          node->get_parameter(ns + "/cam_cy").get_value<double>(),
          node->get_parameter(ns + "/cam_d0", 0.0).get_value<double>(),
          node->get_parameter(ns + "/cam_d1", 0.0).get_value<double>(),
          node->get_parameter(ns + "/cam_d2", 0.0).get_value<double>(),
          node->get_parameter(ns + "/cam_d3", 0.0).get_value<double>()));
    }
    else 
    {
      res = false;
    }
  }

  return res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
