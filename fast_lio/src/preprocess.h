#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUST64,
  MID360,
  PACECAT    // M300 LiDAR
};  //{1, 2, 3, 4, 5}
enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

/* =====================================================================
 * [OPT] 아래 Feature/Surround/E_jump/orgtype은 feature extraction 전용.
 *       M300에서는 feature_extract_enable=false이므로 사용되지 않음.
 *       필요 시 주석 해제.
 * =================================================================== */
#if 0  // [OPT] Feature extraction 관련 타입 비활성화
enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
enum Surround
{
  Prev,
  Next
};
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};
#endif  // [OPT] Feature extraction 관련 타입 비활성화

/* =====================================================================
 * [OPT] 아래 Velodyne/Ouster/Livox 포인트 타입은 M300에서 미사용.
 *       다른 LiDAR 연결 시 주석 해제.
 * =================================================================== */
#if 0  // [OPT] Velodyne 포인트 타입 비활성화
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, time, time)(uint16_t, ring,
                                                                                                        ring))
#endif  // [OPT] Velodyne 포인트 타입 비활성화

#if 0  // [OPT] Ouster 포인트 타입 비활성화
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
#endif  // [OPT] Ouster 포인트 타입 비활성화

#if 0  // [OPT] Livox MID-360 포인트 타입 비활성화
namespace livox_ros
{
typedef struct {
  float x;
  float y;
  float z;
  float reflectivity;
  uint8_t tag;
  uint8_t line;
} LivoxPointXyzrtl;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXyzrtl,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (uint8_t, tag, tag)
    (uint8_t, line, line)
)
#endif  // [OPT] Livox MID-360 포인트 타입 비활성화

// M300 PointCloud2 layout:
// x(f32) y(f32) z(f32) intensity(f32) tag(u8) line(u8) timestamp(f64)
// point_step = 26, timestamp = nanoseconds offset from first point
namespace m300_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace m300_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(m300_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, tag, tag)
    (std::uint8_t, line, line)
    (double, timestamp, timestamp)
)

class Preprocess
{
  public:

  Preprocess();
  ~Preprocess();
  
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // [OPT] feature_enabled 파라미터 제거 (M300에서 미사용)
  // void set(bool feat_en, int lid_type, double bld, int pfilt_num);  // [OPT] 원본
  void set(int lid_type, double bld, int pfilt_num);  // [OPT] 변경

  // [OPT] pl_corn, pl_buff, typess는 feature extraction 전용 — 비활성화
  // PointCloudXYZI pl_full, pl_corn, pl_surf;             // [OPT] 원본
  // PointCloudXYZI pl_buff[128];                           // [OPT] 원본
  // vector<orgtype> typess[128];                           // [OPT] 원본
  PointCloudXYZI pl_full, pl_surf;                          // [OPT] 변경
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  // bool feature_enabled, given_offset_time;               // [OPT] 원본
  bool given_offset_time;                                   // [OPT] 변경

private:
  // [OPT] M300 전용 — 미사용 핸들러 선언 비활성화
  // void oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);     // [OPT] 미사용
  // void velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);   // [OPT] 미사용
  // void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);     // [OPT] 미사용
  void m300_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  // void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);    // [OPT] 미사용

  // [OPT] Feature extraction 관련 함수/변수 비활성화
  // void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  // void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  // int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  // bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  // bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  // int group_size;
  // double disA, disB, inf_bound;
  // double limit_maxmid, limit_midmin, limit_maxmin;
  // double p2l_ratio;
  // double jump_up_limit, jump_down_limit;
  // double cos160;
  // double edgea, edgeb;
  // double smallp_intersect, smallp_ratio;
  // double vx, vy, vz;
};
