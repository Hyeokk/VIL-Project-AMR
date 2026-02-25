// preprocess.cpp
// [OPT] M300 전용 최적화: 미사용 LiDAR 핸들러 및 feature extraction 코드를 #if 0 으로 비활성화
// [OPT] 복원 필요 시 #if 0 → #if 1 로 변경 (preprocess.h의 대응 블록도 함께 해제)
#include "preprocess.h"

// [OPT] 원본: feature extraction 관련 초기화 포함
// Preprocess::Preprocess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
// [OPT] 변경: M300 기본값으로 단순화
Preprocess::Preprocess()
  : lidar_type(PACECAT), blind(0.5), point_filter_num(3),
    N_SCANS(6), SCAN_RATE(10), time_unit(NS),
    given_offset_time(false), time_unit_scale(1.0f)
{
  /* [OPT] 원본에 있던 feature extraction 초기화 — M300 미사용
  inf_bound = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1;
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
  */
}

Preprocess::~Preprocess() {}

// [OPT] 원본: void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
void Preprocess::set(int lid_type, double bld, int pfilt_num)
{
  // feature_enabled = feat_en;   // [OPT] 제거
  lidar_type       = lid_type;
  blind            = bld;
  point_filter_num = pfilt_num;
}

// === M300 전용 핸들러 (활성) ===
// M300 PointCloud2: x,y,z,intensity,tag,line,timestamp(ns)
// timestamp를 curvature(ms)로 변환하여 motion undistortion에 사용
void Preprocess::m300_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();

  pcl::PointCloud<m300_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  // M300 timestamp field is nanoseconds offset from first point
  // FAST-LIO2 curvature unit: ms
  given_offset_time = true;

  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;
    if (range < (blind * blind)) continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.curvature = pl_orig.points[i].timestamp / 1e6;  // ns → ms

    pl_surf.points.push_back(added_pt);
  }
}

// [OPT] process()에서 m300_handler만 호출하도록 단순화
void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                         PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC: time_unit_scale = 1.e3f;  break;
    case MS:  time_unit_scale = 1.f;    break;
    case US:  time_unit_scale = 1.e-3f; break;
    case NS:  time_unit_scale = 1.e-6f; break;
    default:  time_unit_scale = 1.f;    break;
  }

  /* [OPT] 원본 switch — 다른 LiDAR 사용 시 복원
  switch (lidar_type)
  {
    case OUST64:
      oust64_handler(msg);
      break;
    case VELO16:
      velodyne_handler(msg);
      break;
    case MID360:
      mid360_handler(msg);
      break;
    case PACECAT:
      m300_handler(msg);
      break;
    default:
      default_handler(msg);
      break;
  }
  */
  // [OPT] 변경: M300 전용 — 직접 호출
  m300_handler(msg);

  *pcl_out = pl_surf;
}

/* =====================================================================
 * [OPT] 아래는 원본에 있던 미사용 LiDAR 핸들러 및 feature extraction 코드.
 *       M300 프로젝트에서 호출되지 않으므로 #if 0으로 비활성화.
 *       다른 LiDAR 사용 시 #if 1로 변경하고, preprocess.h의 대응 블록도 해제.
 *       원본 전체 코드는 git history 참조:
 *         - oust64_handler()    : Ouster OS-64 핸들러
 *         - velodyne_handler()  : Velodyne VLP-16 핸들러
 *         - mid360_handler()    : Livox MID-360 핸들러
 *         - default_handler()   : 범용 PointCloud2 핸들러
 *         - give_feature()      : Feature point extraction
 *         - plane_judge()       : 평면 판별
 *         - edge_jump_judge()   : 에지 판별
 *         - pub_func()          : 포인트클라우드 publish 헬퍼
 * =================================================================== */
#if 0  // [OPT] 미사용 핸들러/feature extraction 전체 비활성화

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  // ... Ouster OS-64 처리 코드 (원본 git history 참조)
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  // ... Velodyne VLP-16 처리 코드 (원본 git history 참조)
}

void Preprocess::mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  // ... Livox MID-360 처리 코드 (원본 git history 참조)
}

void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  // ... 범용 PointCloud2 처리 코드 (원본 git history 참조)
}

void Preprocess::give_feature(pcl::PointCloud<PointType>& pl, vector<orgtype>& types)
{
  // ... Feature extraction 코드 (원본 git history 참조)
}

void Preprocess::pub_func(PointCloudXYZI& pl, const rclcpp::Time& ct)
{
  // ... publish 헬퍼 (원본 git history 참조)
}

int Preprocess::plane_judge(const PointCloudXYZI& pl, vector<orgtype>& types, uint i_cur, uint& i_nex, Eigen::Vector3d& curr_direct)
{
  // ... 평면 판별 코드 (원본 git history 참조)
  return 0;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI& pl, vector<orgtype>& types, uint i, Surround nor_dir)
{
  // ... 에지 판별 코드 (원본 git history 참조)
  return false;
}

#endif  // [OPT] 미사용 핸들러/feature extraction 전체 비활성화
