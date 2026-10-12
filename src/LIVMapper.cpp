/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <vikit/atan_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/polynomial_camera.h>
#include <chrono>
#include <filesystem>
#include <thread>

namespace
{
std::string cameraParameterName(const std::string &ns, const std::string &name)
{
  return ns.empty() ? name : ns + "." + name;
}

template <typename T>
T cameraParameter(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value)
{
  if (!node->has_parameter(name))
    return node->declare_parameter<T>(name, default_value);

  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

bool loadCamera(const rclcpp::Node::SharedPtr &node, const std::string &ns, vk::AbstractCamera *&camera)
{
  const auto key = [&ns](const std::string &name) { return cameraParameterName(ns, name); };
  const std::string model = cameraParameter<std::string>(node, key("cam_model"), "");

  if (model == "Ocam")
  {
    camera = new vk::OmniCamera(cameraParameter<std::string>(node, key("cam_calib_file"), ""));
  }
  else if (model == "Pinhole")
  {
    camera = new vk::PinholeCamera(
      cameraParameter<int>(node, key("cam_width"), 0),
      cameraParameter<int>(node, key("cam_height"), 0),
      cameraParameter<double>(node, key("scale"), 1.0),
      cameraParameter<double>(node, key("cam_fx"), 0.0),
      cameraParameter<double>(node, key("cam_fy"), 0.0),
      cameraParameter<double>(node, key("cam_cx"), 0.0),
      cameraParameter<double>(node, key("cam_cy"), 0.0),
      cameraParameter<double>(node, key("cam_d0"), 0.0),
      cameraParameter<double>(node, key("cam_d1"), 0.0),
      cameraParameter<double>(node, key("cam_d2"), 0.0),
      cameraParameter<double>(node, key("cam_d3"), 0.0));
  }
  else if (model == "EquidistantCamera")
  {
    camera = new vk::EquidistantCamera(
      cameraParameter<int>(node, key("cam_width"), 0),
      cameraParameter<int>(node, key("cam_height"), 0),
      cameraParameter<double>(node, key("scale"), 1.0),
      cameraParameter<double>(node, key("cam_fx"), 0.0),
      cameraParameter<double>(node, key("cam_fy"), 0.0),
      cameraParameter<double>(node, key("cam_cx"), 0.0),
      cameraParameter<double>(node, key("cam_cy"), 0.0),
      cameraParameter<double>(node, key("k1"), 0.0),
      cameraParameter<double>(node, key("k2"), 0.0),
      cameraParameter<double>(node, key("k3"), 0.0),
      cameraParameter<double>(node, key("k4"), 0.0));
  }
  else if (model == "PolynomialCamera")
  {
    camera = new vk::PolynomialCamera(
      cameraParameter<int>(node, key("cam_width"), 0),
      cameraParameter<int>(node, key("cam_height"), 0),
      cameraParameter<double>(node, key("cam_fx"), 0.0),
      cameraParameter<double>(node, key("cam_fy"), 0.0),
      cameraParameter<double>(node, key("cam_cx"), 0.0),
      cameraParameter<double>(node, key("cam_cy"), 0.0),
      cameraParameter<double>(node, key("cam_skew"), 0.0),
      cameraParameter<double>(node, key("k2"), 0.0),
      cameraParameter<double>(node, key("k3"), 0.0),
      cameraParameter<double>(node, key("k4"), 0.0),
      cameraParameter<double>(node, key("k5"), 0.0),
      cameraParameter<double>(node, key("k6"), 0.0),
      cameraParameter<double>(node, key("k7"), 0.0));
  }
  else if (model == "ATAN")
  {
    camera = new vk::ATANCamera(
      cameraParameter<int>(node, key("cam_width"), 0),
      cameraParameter<int>(node, key("cam_height"), 0),
      cameraParameter<double>(node, key("cam_fx"), 0.0),
      cameraParameter<double>(node, key("cam_fy"), 0.0),
      cameraParameter<double>(node, key("cam_cx"), 0.0),
      cameraParameter<double>(node, key("cam_cy"), 0.0),
      cameraParameter<double>(node, key("cam_d0"), 0.0));
  }
  else
  {
    camera = nullptr;
    return false;
  }

  return true;
}

rclcpp::Time timeFromSeconds(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)));
}

Sophus::SE3 alignPointSets(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &target,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &source)
{
  if (target.empty() || target.size() != source.size())
    return Sophus::SE3();

  Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d source_center = Eigen::Vector3d::Zero();
  for (const auto &point : target)
    target_center += point;
  for (const auto &point : source)
    source_center += point;
  target_center /= static_cast<double>(target.size());
  source_center /= static_cast<double>(source.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < source.size(); ++i)
    covariance += (target[i] - target_center) * (source[i] - source_center).transpose();

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
  if (rotation.determinant() < 0.0)
  {
    Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
    reflection(2, 2) = -1.0;
    rotation = svd.matrixU() * reflection * svd.matrixV().transpose();
  }

  return Sophus::SE3(Sophus::SO3(rotation), target_center - rotation * source_center);
}
} // namespace

LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &nh)
    : nh_(nh), extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = nh->now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &nh)
{
  lid_topic = nh->declare_parameter<string>("common.lid_topic", "/livox/lidar");
  imu_topic = nh->declare_parameter<string>("common.imu_topic", "/livox/imu");
  ros_driver_fix_en = nh->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  img_en = nh->declare_parameter<int>("common.img_en", 1);
  lidar_en = nh->declare_parameter<int>("common.lidar_en", 1);
  img_topic = nh->declare_parameter<string>("common.img_topic", "/left_camera/image");

  normal_en = nh->declare_parameter<bool>("vio.normal_en", true);
  inverse_composition_en = nh->declare_parameter<bool>("vio.inverse_composition_en", false);
  max_iterations = nh->declare_parameter<int>("vio.max_iterations", 5);
  IMG_POINT_COV = nh->declare_parameter<double>("vio.img_point_cov", 100.0);
  raycast_en = nh->declare_parameter<bool>("vio.raycast_en", false);
  exposure_estimate_en = nh->declare_parameter<bool>("vio.exposure_estimate_en", true);
  inv_expo_cov = nh->declare_parameter<double>("vio.inv_expo_cov", 0.2);
  grid_size = nh->declare_parameter<int>("vio.grid_size", 5);
  grid_n_height = nh->declare_parameter<int>("vio.grid_n_height", 17);
  patch_pyrimid_level = nh->declare_parameter<int>("vio.patch_pyrimid_level", 3);
  patch_size = nh->declare_parameter<int>("vio.patch_size", 8);
  outlier_threshold = nh->declare_parameter<double>("vio.outlier_threshold", 1000.0);

  exposure_time_init = nh->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  img_time_offset = nh->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  imu_time_offset = nh->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  lidar_time_offset = nh->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  imu_prop_enable = nh->declare_parameter<bool>("uav.imu_rate_odom", false);
  gravity_align_en = nh->declare_parameter<bool>("uav.gravity_align_en", false);

  seq_name = nh->declare_parameter<string>("evo.seq_name", "01");
  pose_output_en = nh->declare_parameter<bool>("evo.pose_output_en", false);
  gyr_cov = nh->declare_parameter<double>("imu.gyr_cov", 1.0);
  acc_cov = nh->declare_parameter<double>("imu.acc_cov", 1.0);
  imu_int_frame = nh->declare_parameter<int>("imu.imu_int_frame", 3);
  imu_en = nh->declare_parameter<bool>("imu.imu_en", false);
  gravity_est_en = nh->declare_parameter<bool>("imu.gravity_est_en", true);
  ba_bg_est_en = nh->declare_parameter<bool>("imu.ba_bg_est_en", true);

  p_pre->blind = nh->declare_parameter<double>("preprocess.blind", 0.01);
  filter_size_surf_min = nh->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  hilti_en = nh->declare_parameter<bool>("preprocess.hilti_en", false);
  p_pre->lidar_type = nh->declare_parameter<int>("preprocess.lidar_type", AVIA);
  p_pre->N_SCANS = nh->declare_parameter<int>("preprocess.scan_line", 6);
  p_pre->point_filter_num = nh->declare_parameter<int>("preprocess.point_filter_num", 3);
  p_pre->feature_enabled = nh->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  pcd_save_interval = nh->declare_parameter<int>("pcd_save.interval", -1);
  pcd_save_en = nh->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  pcd_save_type = nh->declare_parameter<int>("pcd_save.type", 0);
  img_save_en = nh->declare_parameter<bool>("image_save.img_save_en", false);
  img_save_interval = nh->declare_parameter<int>("image_save.interval", 1);
  colmap_output_en = nh->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  filter_size_pcd = nh->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  extrinT = nh->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>());
  extrinR = nh->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>());
  cameraextrinT = nh->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>());
  cameraextrinR = nh->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>());
  plot_time = nh->declare_parameter<double>("debug.plot_time", -10.0);
  frame_cnt = nh->declare_parameter<int>("debug.frame_cnt", 6);

  blind_rgb_points = nh->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  pub_scan_num = nh->declare_parameter<int>("publish.pub_scan_num", 1);
  pub_effect_point_en = nh->declare_parameter<bool>("publish.pub_effect_point_en", false);
  dense_map_en = nh->declare_parameter<bool>("publish.dense_map_en", false);

  rtk_en = nh->declare_parameter<bool>("gps.gps_en", false);
  gps_topic = nh->declare_parameter<std::string>("gps.gps_topic", "/ublox_driver/receiver_pvt");
  T_I_R = nh->declare_parameter<std::vector<double>>(
      "gps.extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
  debug_mode = nh->declare_parameter<bool>("gps.debug_mode", false);
  output_directory = nh->declare_parameter<std::string>(
      "output.directory", std::string("/tmp/fast_livgo_rtk"));

  rtk_pcd_directory = output_directory + "/debug/pcd";
  rtk_save_file = output_directory + "/debug/rtk.txt";
  imu_save_file = output_directory + "/debug/imu.txt";
  odom_save_file = output_directory + "/debug/odom.txt";
  cov_save_file = output_directory + "/debug/cov.txt";

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents()
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!loadCamera(nh_, "camera", vio_manager->cam))
    throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->output_directory = output_directory;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  p_imu->configure_debug_output(debug_mode, imu_save_file);

  if (!imu_en)
    p_imu->disable_imu();
  if (!gravity_est_en)
    p_imu->disable_gravity_est();
  if (!ba_bg_est_en)
    p_imu->disable_bias_est();
  if (!exposure_estimate_en)
    p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO
                                                    : ONLY_LO;
}

void LIVMapper::initializeFiles()
{
  if (debug_mode)
  {
    std::filesystem::create_directories(output_directory + "/debug");
    std::filesystem::create_directories(rtk_pcd_directory);
  }
  std::filesystem::create_directories(output_directory + "/TUM");
  std::filesystem::create_directories(output_directory + "/vel");
  std::filesystem::create_directories(output_directory + "/global_pcd");
  std::filesystem::create_directories(output_directory + "/scan_pcd");

  if (pcd_save_en)
    std::filesystem::create_directories(output_directory + "/pcd");
  if (img_save_en)
    std::filesystem::create_directories(output_directory + "/image");
  if (colmap_output_en)
  {
    std::filesystem::remove_all(output_directory + "/Colmap");
    std::filesystem::create_directories(output_directory + "/Colmap/images");
    std::filesystem::create_directories(output_directory + "/Colmap/sparse/0");
  }
  if (colmap_output_en)
    fout_points.open(output_directory + "/Colmap/sparse/0/points3D.txt", std::ios::out);
  if (pcd_save_en)
    fout_lidar_pos.open(output_directory + "/pcd/lidar_poses.txt", std::ios::out);
  if (img_save_en)
    fout_visual_pos.open(output_directory + "/image/image_poses.txt", std::ios::out);
  if (debug_mode)
  {
    fout_pre.open(output_directory + "/debug/mat_pre.txt", std::ios::out);
    fout_out.open(output_directory + "/debug/mat_out.txt", std::ios::out);
  }
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &nh, image_transport::ImageTransport &it)
{
  if (p_pre->lidar_type == AVIA)
  {
    sub_lv_pcl = nh->create_subscription<fast_livgo::msg::CustomMsg>(
        lid_topic, 200000,
        std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  }
  else
  {
    sub_pcl = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, 200000,
        std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 200000,
      std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  sub_img = nh->create_subscription<sensor_msgs::msg::Image>(
      img_topic, 200000,
      std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
  if (rtk_en)
  {
    sub_rtk = nh->create_subscription<gnss_comm::msg::GnssPVTSolnMsg>(
        gps_topic, rclcpp::SensorDataQoS(),
        std::bind(&LIVMapper::rtk_cbk, this, std::placeholders::_1));
  }

  pub_rtk = nh->create_publisher<nav_msgs::msg::Odometry>("/gps/odometry", 100);
  pub_rtk_backend_odom = nh->create_publisher<nav_msgs::msg::Odometry>("/odometry/fast_livgo2", 100);
  pub_rtk_backend_cloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/synced_cloud", 100);

  pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubLaserCloudEffect = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 10);
  mavros_pose_publisher = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = nh->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = nh->create_wall_timer(std::chrono::milliseconds(4), std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame()
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment()
{
  if (!p_imu->imu_need_init && !gravity_align_finished)
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu()
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort, T_G_to_W);

  if (gravity_align_en)
    gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping()
{
  switch (LidarMeasures.lio_vio_flg)
  {
  case VIO:
    handleVIO();
    break;
  case LIO:
  case LO:
    handleLIO();
    break;
  }
}

void LIVMapper::handleVIO()
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;

  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr))
    return;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1))
  {
    vio_manager->plot_flag = true;
  }
  else
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO()
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;

  if (feats_undistort->empty() || (feats_undistort == nullptr))
    return;

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited)
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  rtk_good = false;
  V3D rtk_position = V3D::Zero();
  if (rtk_en && rtk_ini && !LidarMeasures.measures.empty())
  {
    const RTK &rtk = LidarMeasures.measures.back().rtk;
    if (rtk.timestamp > 0.00001)
    {
      // Process2 has already transformed the RTK sample and propagated it to
      // the current LIO update time.
      rtk_position = rtk.p;
      rtk_good = true;
    }
  }
  voxelmap_manager->StateEstimation(state_propagat, rtk_good, rtk_position);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en)
  {
    static bool pos_opend = false;
    std::ofstream evoFile;
    if (!pos_opend)
    {
      evoFile.open(output_directory + "/TUM/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open())
        RCLCPP_ERROR(nh_->get_logger(), "open fail\n");
    }
    else
    {
      evoFile.open(output_directory + "/TUM/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open())
        RCLCPP_ERROR(nh_->get_logger(), "open fail\n");
    }
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  {
    tf2::Quaternion q;
    q.setRPY(euler_cur(0), euler_cur(1), euler_cur(2));
    geoQuat = tf2::toMsg(q);
  }
  publish_odometry(pubOdomAftMapped);

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++)
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  _pv_list = voxelmap_manager->pv_list_;

  if (voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }

  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++)
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en)
    publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_)
    voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);


  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD()
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0)
  {
    std::string raw_points_dir = output_directory + "/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = output_directory + "/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);

      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;

      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if (colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i)
        {
          const auto &point = downsampled_cloud->points[i];
          fout_points << i << " "
                      << std::fixed << std::setprecision(6)
                      << point.x << " " << point.y << " " << point.z << " "
                      << static_cast<int>(point.r) << " "
                      << static_cast<int>(point.g) << " "
                      << static_cast<int>(point.b) << " "
                      << 0 << std::endl;
        }
      }
    }
    else
    {
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run()
{
  const auto idle_period = std::chrono::microseconds(200);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
    if (!rclcpp::ok())
    {
      break;
    }
    if (!sync_packages(LidarMeasures))
    {
      std::this_thread::sleep_for(idle_period);
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once)
  {
    return;
  }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && rclcpp::Time(prop_imu_buffer.front().header.stamp).seconds() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = rclcpp::Time(prop_imu_buffer[i].header.stamp).seconds() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = rclcpp::Time(newest_imu.header.stamp).seconds() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T>
void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T>
Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en)
    return;
  mtx_buffer.lock();

  double cur_head_time = rclcpp::Time(msg->header.stamp).seconds() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(nh_->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const fast_livgo::msg::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en)
    return;
  mtx_buffer.lock();
  fast_livgo::msg::CustomMsg::Ptr msg(new fast_livgo::msg::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - rclcpp::Time(msg->header.stamp).seconds()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - rclcpp::Time(msg->header.stamp).seconds();
    RCLCPP_DEBUG_THROTTLE(
        nh_->get_logger(), *nh_->get_clock(), 5000,
        "IMU and LiDAR hard time lag: %.6f", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = rclcpp::Time(msg->header.stamp).seconds();
  RCLCPP_DEBUG(nh_->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(nh_->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::rtk_cbk(const gnss_comm::msg::GnssPVTSolnMsg::ConstSharedPtr &gps_msg)
{
  static bool first_gps = true;
  if (first_gps)
  {
    gps_trans_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    first_gps = false;
    if (debug_mode)
    {
      std::ofstream gps_file(rtk_save_file, std::ios::out);
      gps_file << "# timestamp x y z vx vy vz h_acc v_acc\n";
    }
  }

  Eigen::Vector3d local_position;
  gps_trans_.Forward(
      gps_msg->latitude, gps_msg->longitude, gps_msg->altitude,
      local_position.x(), local_position.y(), local_position.z());

  constexpr double gps_epoch_unix_time = 315964800.0;
  constexpr double seconds_per_week = 604800.0;
  constexpr double leap_seconds = 18.0;
  const double gps_seconds =
      static_cast<double>(gps_msg->time.week) * seconds_per_week + gps_msg->time.tow;
  const double timestamp = gps_seconds + gps_epoch_unix_time - leap_seconds;
  const rclcpp::Time stamp = timeFromSeconds(timestamp);

  RTK rtk;
  rtk.timestamp = timestamp - 0.001;
  rtk.p = local_position;
  rtk.v << gps_msg->vel_e, gps_msg->vel_n, -gps_msg->vel_d;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    rtk_buffer.push_back(rtk);
  }

  if (!rtk_ini && rtk_en && local_position.norm() > 50.0)
  {
    initializeRTK();
    rtk_ini = true;
    RCLCPP_INFO(nh_->get_logger(), "RTK-to-LIVO alignment initialized");
  }

  nav_msgs::msg::Odometry gps_odom;
  gps_odom.header.stamp = stamp;
  gps_odom.header.frame_id = "camera_init";
  gps_odom.pose.pose.position.x = local_position.x();
  gps_odom.pose.pose.position.y = local_position.y();
  gps_odom.pose.pose.position.z = local_position.z();
  gps_odom.pose.pose.orientation.w = 1.0;
  gps_odom.twist.twist.linear.x = rtk.v.x();
  gps_odom.twist.twist.linear.y = rtk.v.y();
  gps_odom.twist.twist.linear.z = rtk.v.z();
  gps_odom.pose.covariance[0] = gps_msg->h_acc;
  gps_odom.pose.covariance[7] = gps_msg->h_acc;
  gps_odom.pose.covariance[14] = gps_msg->v_acc;
  pub_rtk->publish(gps_odom);

  if (debug_mode)
  {
    std::ofstream gps_file(rtk_save_file, std::ios::app);
    gps_file << std::fixed << std::setprecision(6)
             << timestamp << " " << local_position.transpose() << " "
             << rtk.v.transpose() << " " << gps_msg->h_acc << " "
             << gps_msg->v_acc << std::endl;
  }
}

void LIVMapper::initializeRTK()
{
  constexpr double time_sync_threshold = 0.05;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> livo_points;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rtk_points;
  auto livo_it = livo_state_buffer.begin();

  for (const RTK &rtk : rtk_buffer)
  {
    while (livo_it != livo_state_buffer.end() &&
           (*livo_it)[0] < rtk.timestamp - time_sync_threshold)
      ++livo_it;
    if (livo_it == livo_state_buffer.end())
      break;

    auto best_it = livo_it;
    auto next_it = std::next(livo_it);
    if (next_it != livo_state_buffer.end() &&
        std::abs((*next_it)[0] - rtk.timestamp) < std::abs((*best_it)[0] - rtk.timestamp))
      best_it = next_it;

    if (best_it->size() >= 4 &&
        std::abs((*best_it)[0] - rtk.timestamp) < time_sync_threshold)
    {
      rtk_points.push_back(rtk.p);
      livo_points.emplace_back((*best_it)[1], (*best_it)[2], (*best_it)[3]);
    }
  }

  if (rtk_points.size() < 3)
  {
    RCLCPP_WARN(
        nh_->get_logger(),
        "RTK alignment has only %zu correspondences", rtk_points.size());
  }

  T_W_to_G = alignPointSets(rtk_points, livo_points);
  T_G_to_W = T_W_to_G.inverse();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstPtr &msg_in)
{
  if (!imu_en)
    return;

  if (last_timestamp_lidar < 0.0)
    return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = rclcpp::Time(rclcpp::Time(msg->header.stamp).nanoseconds() - (int64_t)(imu_time_offset * 1e9));
  double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  // RCLCPP_INFO(nh_->get_logger(), "%f - %f, diff %f", last_timestamp_lidar, timestamp, last_timestamp_lidar - last_timestamp_imu);
  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN_THROTTLE(
        nh_->get_logger(), *nh_->get_clock(), 5000,
        "IMU and LiDAR not synced; delta time: %.6f", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en)
    timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = rclcpp::Time((int64_t)(timestamp * 1e9));

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(nh_->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init)
    {
      prop_imu_buffer.push_back(*msg);
    }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstPtr &msg_in)
{
  if (!img_en)
    return;
  sensor_msgs::msg::Image::Ptr msg(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = rclcpp::Time(msg->header.stamp).seconds() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001)
    return;
  RCLCPP_DEBUG(nh_->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0)
    return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(nh_->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(nh_->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en)
    return false;
  if (img_buffer.empty() && img_en)
    return false;
  if (imu_buffer.empty() && imu_en)
    return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0)
      meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1)
        return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                 // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true; // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    keyframe_time = m.lio_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > meas.lidar_frame_end_time)
        break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      keyframe_time = img_capture_time;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0)
        meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = rclcpp::Time(imu_buffer.back()->header.stamp).seconds();

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(nh_->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      if (rtk_en && rtk_ini)
      {
        while (!rtk_buffer.empty())
        {
          if (rtk_buffer.front().timestamp > m.lio_time)
            break;
          if (rtk_buffer.front().timestamp > meas.last_lio_update_time)
          {
            m.rtk = rtk_buffer.front();
            break;
          }
          rtk_buffer.pop_front();
        }
      }
      while (!imu_buffer.empty())
      {
        if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > m.lio_time)
          break;

        if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > meas.last_lio_update_time)
          m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time)
          break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed)
    {
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())
        return false;
      meas.lidar = lid_raw_data_buffer.front();                                                                   // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                 // generate lidar_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    keyframe_time = m.lio_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false;  // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(nh_->get_logger(), "out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = nh_->now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty())
    return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 0;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if (pub_num >= pub_scan_num)
    {
      pub_num = 0;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w));
        if (pf[2] < 0)
          continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255;
          // else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255;
          // else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255;
          // else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points)
            laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  bool publish_cloud = false;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
    publish_cloud = true;
  }
  else if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  {
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
    publish_cloud = true;
  }
  if (publish_cloud)
  {
    laserCloudmsg.header.stamp = nh_->now(); //.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);
  }

  const bool publish_backend_frame =
      (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO && !laserCloudWorldRGB->empty()) ||
      ((slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO) && !pcl_w_wait_pub->empty());
  if (publish_backend_frame)
  {
    sensor_msgs::msg::PointCloud2 synced_cloud;
    if (slam_mode_ == LIVO)
    {
      PointCloudXYZRGB::Ptr body_cloud(new PointCloudXYZRGB());
      body_cloud->reserve(laserCloudWorldRGB->size());
      for (const auto &world_point : laserCloudWorldRGB->points)
      {
        const V3D body_position =
            _state.rot_end.transpose() *
            (V3D(world_point.x, world_point.y, world_point.z) - _state.pos_end);
        PointTypeRGB body_point = world_point;
        body_point.x = body_position.x();
        body_point.y = body_position.y();
        body_point.z = body_position.z();
        body_cloud->push_back(body_point);
      }
      pcl::toROSMsg(*body_cloud, synced_cloud);

      if (debug_mode && !body_cloud->empty())
      {
        std::ostringstream filename;
        filename << rtk_pcd_directory << "/" << std::setfill('0')
                 << std::setw(6) << rtk_pcd_file_index++ << ".pcd";
        pcl::io::savePCDFileBinary(filename.str(), *body_cloud);
      }
    }
    else
    {
      PointCloudXYZI::Ptr body_cloud(new PointCloudXYZI());
      body_cloud->reserve(pcl_w_wait_pub->size());
      for (const auto &world_point : pcl_w_wait_pub->points)
      {
        const V3D body_position =
            _state.rot_end.transpose() *
            (V3D(world_point.x, world_point.y, world_point.z) - _state.pos_end);
        PointType body_point = world_point;
        body_point.x = body_position.x();
        body_point.y = body_position.y();
        body_point.z = body_position.z();
        body_cloud->push_back(body_point);
      }
      pcl::toROSMsg(*body_cloud, synced_cloud);

      if (debug_mode && !body_cloud->empty())
      {
        std::ostringstream filename;
        filename << rtk_pcd_directory << "/" << std::setfill('0')
                 << std::setw(6) << rtk_pcd_file_index++ << ".pcd";
        pcl::io::savePCDFileBinary(filename.str(), *body_cloud);
      }
    }

    const rclcpp::Time frame_stamp = timeFromSeconds(keyframe_time);
    synced_cloud.header.stamp = frame_stamp;
    synced_cloud.header.frame_id = "body";

    nav_msgs::msg::Odometry backend_odom;
    backend_odom.header.stamp = frame_stamp;
    backend_odom.header.frame_id = "camera_init";
    backend_odom.child_frame_id = "body";
    backend_odom.pose.pose.position.x = _state.pos_end.x();
    backend_odom.pose.pose.position.y = _state.pos_end.y();
    backend_odom.pose.pose.position.z = _state.pos_end.z();
    const Eigen::Quaterniond orientation(_state.rot_end);
    backend_odom.pose.pose.orientation.w = orientation.w();
    backend_odom.pose.pose.orientation.x = orientation.x();
    backend_odom.pose.pose.orientation.y = orientation.y();
    backend_odom.pose.pose.orientation.z = orientation.z();

    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance(
        backend_odom.pose.covariance.data());
    pose_covariance.setZero();
    pose_covariance.block<3, 3>(0, 0) = _state.cov.block<3, 3>(3, 3);
    pose_covariance.block<3, 3>(3, 3) = _state.cov.block<3, 3>(0, 0);
    pose_covariance.block<3, 3>(0, 3) = _state.cov.block<3, 3>(3, 0);
    pose_covariance.block<3, 3>(3, 0) = _state.cov.block<3, 3>(0, 3);

    pub_rtk_backend_odom->publish(backend_odom);
    pub_rtk_backend_cloud->publish(synced_cloud);
    livo_state_buffer.push_back(
        {keyframe_time, _state.pos_end.x(), _state.pos_end.y(), _state.pos_end.z()});

    if (debug_mode)
    {
      std::ofstream odom_file(odom_save_file, std::ios::app);
      odom_file << std::fixed << std::setprecision(6)
                << keyframe_time << " " << _state.pos_end.transpose() << " "
                << orientation.x() << " " << orientation.y() << " "
                << orientation.z() << " " << orientation.w() << std::endl;
      std::ofstream covariance_file(cov_save_file, std::ios::app);
      covariance_file << std::fixed << std::setprecision(6) << keyframe_time;
      for (double value : backend_odom.pose.covariance)
        covariance_file << " " << value;
      covariance_file << std::endl;
    }
  }

  /**************** save map ****************/
  /* Make sure enough memory is available; PCD saving affects real-time performance. */
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
          scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(output_directory + "/pcd/" + ss_time.str() + ".pcd");

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if (pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos << LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
                     << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(output_directory + "/image/" + ss_time.str() + ".png", vio_manager->img_rgb);

      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
                      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if (laserCloudWorldRGB->size() > 0)
    PointCloudXYZI().swap(*pcl_wait_pub);
  if (LidarMeasures.lio_vio_flg == VIO)
    PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = nh_->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T>
void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = nh_->now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static tf2_ros::TransformBroadcaster br(*nh_);
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = odomAftMapped.header.stamp;
  t.header.frame_id = "camera_init";
  t.child_frame_id = "aft_mapped";
  tf2::toMsg(transform, t.transform);
  br.sendTransform(t);

  // br.sendTransform(tf2::toMsg(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = nh_->now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = nh_->now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
