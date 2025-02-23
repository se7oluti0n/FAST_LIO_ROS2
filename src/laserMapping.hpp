#pragma once

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

class LaserMappingNode: public rclcpp::Node
{
public:

    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LaserMappingNode();
private:
  void declare_parameters();
  void get_parameters();
  void timer_callback();
  void map_publish_callback();
  void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

  void lasermap_fov_segment();
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);
  void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in);
  bool sync_packages(MeasureGroup &meas);
  void map_incremental();
  void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull);
  void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);
  void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect);
  void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
  void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);
  void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
private:
inline void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}
  inline void SigHandle(int sig)
  {
      flg_exit = true;
      std::cout << "catch sig %d" << sig << std::endl;
      sig_buffer.notify_all();
      rclcpp::shutdown();
  }

  inline void dump_lio_state_to_log(FILE *fp)  
  {
      V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
      fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
      fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
      fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
      fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
      fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
      fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
      fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
      fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
      fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
      fprintf(fp, "\r\n");  
      fflush(fp);
  }

  inline void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
  {
      V3D p_body(pi->x, pi->y, pi->z);
      V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

      po->x = p_global(0);
      po->y = p_global(1);
      po->z = p_global(2);
      po->intensity = pi->intensity;
  }


  inline void pointBodyToWorld(PointType const * const pi, PointType * const po)
  {
      V3D p_body(pi->x, pi->y, pi->z);
      V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

      po->x = p_global(0);
      po->y = p_global(1);
      po->z = p_global(2);
      po->intensity = pi->intensity;
  }

  template<typename T>
  void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
  {
      V3D p_body(pi[0], pi[1], pi[2]);
      V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

      po[0] = p_global(0);
      po[1] = p_global(1);
      po[2] = p_global(2);
  }

  inline void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
  {
      V3D p_body(pi->x, pi->y, pi->z);
      V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

      po->x = p_global(0);
      po->y = p_global(1);
      po->z = p_global(2);
      po->intensity = pi->intensity;
  }

  inline void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
  {
      V3D p_body_lidar(pi->x, pi->y, pi->z);
      V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

      po->x = p_body_imu(0);
      po->y = p_body_imu(1);
      po->z = p_body_imu(2);
      po->intensity = pi->intensity;
  }

  inline void points_cache_collect()
  {
      PointVector points_history;
      ikdtree.acquire_removed_points(points_history);
      // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
  }

  /*** Time Log Variables ***/
  double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
  double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;
  int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
  bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
  /**************************/

  float res_last[100000] = {0.0};
  float DET_RANGE = 300.0f;
  const float MOV_THRESHOLD = 1.5f;
  double time_diff_lidar_to_imu = 0.0;

  mutex mtx_buffer;
  condition_variable sig_buffer;

  string root_dir = ROOT_DIR;
  string map_file_path, lid_topic, imu_topic;

  double res_mean_last = 0.05, total_residual = 0.0;
  double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
  double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
  int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
  int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
  bool   point_selected_surf[100000] = {0};
  bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
  bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
  bool    is_first_lidar = true;

  vector<vector<int>>  pointSearchInd_surf; 
  vector<BoxPointType> cub_needrm;
  vector<PointVector>  Nearest_Points; 
  // vector<double>       extrinT(3, 0.0);
  // vector<double>       extrinR(9, 0.0);
  vector<double>       extrinT;
  vector<double>       extrinR;
  deque<double>                     time_buffer;
  deque<PointCloudXYZI::Ptr>        lidar_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

  // PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
  // PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
  // PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
  // PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
  // PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
  // PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
  // PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
  PointCloudXYZI::Ptr featsFromMap;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr normvec;
  PointCloudXYZI::Ptr laserCloudOri;
  PointCloudXYZI::Ptr corr_normvect;
  PointCloudXYZI::Ptr _featsArray;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;

  KD_TREE<PointType> ikdtree;

  // V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
  // V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
  V3F XAxisPoint_body;
  V3F XAxisPoint_world;
  V3D euler_cur;
  // V3D position_last(Zero3d);
  // V3D Lidar_T_wrt_IMU(Zero3d);
  // M3D Lidar_R_wrt_IMU(Eye3d);
  V3D position_last;
  V3D Lidar_T_wrt_IMU;
  M3D Lidar_R_wrt_IMU;

  /*** EKF inputs and output ***/
  MeasureGroup Measures;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;
  vect3 pos_lid;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  // shared_ptr<Preprocess> p_pre(new Preprocess());
  // shared_ptr<ImuProcess> p_imu(new ImuProcess());
  shared_ptr<Preprocess> p_pre;
  shared_ptr<ImuProcess> p_imu;


  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;
  // PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
  // PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_save;
  int process_increments = 0;
  double lidar_mean_scantime = 0.0;
  int    scan_num = 0;
  double timediff_lidar_wrt_imu = 0.0;
  bool   timediff_set_flg = false;
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};
