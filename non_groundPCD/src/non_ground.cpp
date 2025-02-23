#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <open3d/Open3D.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cfloat>
#include <array>
#include <vector>

using namespace std::chrono_literals;

class PointCloudProcessor : public rclcpp::Node {
public:
  PointCloudProcessor() : Node("pointcloud_processor") {
    // 声明参数
    this->declare_parameter("input_path", "/home/hy/wust_nav/src/bringup_nav/pcd/simulation/rmuc_2025.pcd");
    this->declare_parameter("roll", 3.1415926);
    this->declare_parameter("pitch", 0.0);
    this->declare_parameter("yaw", 0.0);
    this->declare_parameter("z_min", -1.0);
    this->declare_parameter("z_max", 0.5);
    this->declare_parameter("max_slope_deg", 15.0);
    this->declare_parameter("max_attempts", 4);
    this->declare_parameter("distance_threshold", 0.1);
    this->declare_parameter("use_projection", true);
    this->declare_parameter("frame_id", "map");

    // 创建发布者
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_cloud", 10);
    non_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_cloud", 10);
    projected_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("projected_cloud", 10);

    // 处理点云
    process_point_cloud();
  }

private:
  void process_point_cloud() {
    RCLCPP_INFO(this->get_logger(), "Starting point cloud processing");

    // 获取参数
    auto input_path = this->get_parameter("input_path").as_string();
    auto roll = this->get_parameter("roll").as_double();
    auto pitch = this->get_parameter("pitch").as_double();
    auto yaw = this->get_parameter("yaw").as_double();
    auto frame_id = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: input_path=%s, roll=%.2f, pitch=%.2f, yaw=%.2f, frame_id=%s",
                input_path.c_str(), roll, pitch, yaw, frame_id.c_str());

    // 读取点云
    RCLCPP_INFO(this->get_logger(), "Reading point cloud from file: %s", input_path.c_str());
    auto pcd = open3d::io::CreatePointCloudFromFile(input_path);
    if (!pcd) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read point cloud file");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Point cloud loaded successfully, total points: %zu", pcd->points_.size());

    // 应用旋转
    RCLCPP_INFO(this->get_logger(), "Applying rotation to point cloud: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
    auto rotated_pcd = rotate_pcd(*pcd, roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "Rotation applied successfully");

    // 地面分割参数
    GroundSegmentationParams params;
    params.z_min = this->get_parameter("z_min").as_double();
    params.z_max = this->get_parameter("z_max").as_double();
    params.max_slope_deg = this->get_parameter("max_slope_deg").as_double();
    params.max_attempts = this->get_parameter("max_attempts").as_int();
    params.distance_threshold = this->get_parameter("distance_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "Ground segmentation parameters: z_min=%.2f, z_max=%.2f, max_slope_deg=%.2f, max_attempts=%d, distance_threshold=%.2f",
                params.z_min, params.z_max, params.max_slope_deg, params.max_attempts, params.distance_threshold);

    // 执行地面分割
    RCLCPP_INFO(this->get_logger(), "Starting ground segmentation");
    auto [ground, non_ground] = extract_ground(rotated_pcd, params);
    RCLCPP_INFO(this->get_logger(), "Ground segmentation completed: ground points=%zu, non-ground points=%zu",
                ground->points_.size(), non_ground->points_.size());

    // 保存地面点云
    if (!ground->points_.empty()) {
      std::string ground_path = "ground_cloud.pcd";
      open3d::io::WritePointCloud(ground_path, *ground);
      RCLCPP_INFO(this->get_logger(), "Ground cloud saved to %s", ground_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Ground cloud is empty, not saved");
    }

    // 保存非地面点云
    if (!non_ground->points_.empty()) {
      std::string non_ground_path = "non_ground_cloud.pcd";
      open3d::io::WritePointCloud(non_ground_path, *non_ground);
      RCLCPP_INFO(this->get_logger(), "Non-ground cloud saved to %s", non_ground_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Non-ground cloud is empty, not saved");
    }

    // 发布点云
    RCLCPP_INFO(this->get_logger(), "Publishing ground and non-ground clouds");
    publish_cloud(*ground_pub_, *ground, frame_id, {0.0f, 1.0f, 0.0f});
    publish_cloud(*non_ground_pub_, *non_ground, frame_id, {1.0f, 0.0f, 0.0f});

    // 投影处理
    if (this->get_parameter("use_projection").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "Starting projection to ground plane");
      auto projected = project_to_ground_plane(*non_ground, *ground);
      if (projected) {
        RCLCPP_INFO(this->get_logger(), "Projection completed: projected points=%zu", projected->points_.size());

        // 保存投影点云
        if (!projected->points_.empty()) {
          std::string projected_path = "projected_cloud.pcd";
          open3d::io::WritePointCloud(projected_path, *projected);
          RCLCPP_INFO(this->get_logger(), "Projected cloud saved to %s", projected_path.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Projected cloud is empty, not saved");
        }
        publish_cloud(*projected_pub_, *projected, frame_id, {0.0f, 0.0f, 1.0f});
      }
    }

    RCLCPP_INFO(this->get_logger(), "Point cloud processing completed");
  }
  open3d::geometry::PointCloud rotate_pcd(const open3d::geometry::PointCloud& pcd, 
                                        double roll, double pitch, double yaw) {
    Eigen::Matrix3d rotation = (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
    ).toRotationMatrix();
    
    open3d::geometry::PointCloud rotated = pcd;
    rotated.Rotate(rotation, Eigen::Vector3d::Zero());
    return rotated;
  }

  struct GroundSegmentationParams {
    double z_min;
    double z_max;
    double max_slope_deg;
    int max_attempts;
    double distance_threshold;
  };

  std::pair<std::shared_ptr<open3d::geometry::PointCloud>,
            std::shared_ptr<open3d::geometry::PointCloud>>
  extract_ground(const open3d::geometry::PointCloud& pcd, 
                const GroundSegmentationParams& params) {
    auto cropped = pcd.Crop(open3d::geometry::AxisAlignedBoundingBox(
      Eigen::Vector3d(-DBL_MAX, -DBL_MAX, params.z_min),
      Eigen::Vector3d(DBL_MAX, DBL_MAX, params.z_max))
    );

    auto ground = std::make_shared<open3d::geometry::PointCloud>();
    auto remaining = std::make_shared<open3d::geometry::PointCloud>(*cropped);

    double normal_z_min = cos(params.max_slope_deg * M_PI / 180.0);

    for (int i = 0; i < params.max_attempts; ++i) {
      auto [plane_model, inliers] = remaining->SegmentPlane(
        params.distance_threshold, 3, 1000);

      if (inliers.empty()) break;

      auto candidate = remaining->SelectByIndex(inliers);
      Eigen::Vector4d plane = plane_model;

      Eigen::Vector3d normal(plane(0), plane(1), plane(2));
      normal.normalize();
      double normal_z = std::abs(normal.z());

      double avg_z = std::accumulate(candidate->points_.begin(), candidate->points_.end(), 0.0,
        [](double sum, const Eigen::Vector3d& p) { return sum + p.z(); }
      ) / candidate->points_.size();

      if (normal_z >= normal_z_min && avg_z >= params.z_min && avg_z <= params.z_max) {
        *ground += *candidate;
        remaining = remaining->SelectByIndex(inliers, true);
      } else {
        remaining = remaining->SelectByIndex(inliers, true);
      }
    }

    // 处理超出z范围的点
    auto full_points = pcd.points_;
    std::vector<size_t> non_ground_indices;
    for (size_t i = 0; i < full_points.size(); ++i) {
      if (full_points[i].z() < params.z_min || full_points[i].z() > params.z_max) {
        non_ground_indices.push_back(i);
      }
    }
    auto non_ground = pcd.SelectByIndex(non_ground_indices);
    non_ground->points_.insert(non_ground->points_.end(), 
                             remaining->points_.begin(), remaining->points_.end());

    return {ground, non_ground};
  }

  std::shared_ptr<open3d::geometry::PointCloud> project_to_ground_plane(
    const open3d::geometry::PointCloud& non_ground,
    const open3d::geometry::PointCloud& ground) {
    
    if (ground.points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Ground cloud is empty");
      return nullptr;
    }

    // 使用 for 循环计算地面点云的中心点
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : ground.points_) {
      centroid += p;
    }
    centroid /= ground.points_.size();

    // 计算协方差矩阵
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : ground.points_) {
      Eigen::Vector3d d = p - centroid;
      cov += d * d.transpose();
    }
    cov /= ground.points_.size();

    // 求解特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (normal.z() < 0) normal = -normal; // 确保法向量朝上

    // 计算平面方程参数 d
    double d = -normal.dot(centroid);

    // 将非地面点云投影到地面平面
    auto projected = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& p : non_ground.points_) {
      double dist = normal.dot(p) + d; // 点到平面的距离
      Eigen::Vector3d proj = p - normal * dist; // 投影点
      projected->points_.push_back(proj);
    }

    return projected;
  }

  void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub,
                    const open3d::geometry::PointCloud& pcd,
                    const std::string& frame_id,
                    const std::array<float, 3>& color) {
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    for (const auto& p : pcd.points_) {
      pcl::PointXYZRGB point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      point.r = color[0] * 255;
      point.g = color[1] * 255;
      point.b = color[2] * 255;
      pcl_cloud.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    pub.publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
