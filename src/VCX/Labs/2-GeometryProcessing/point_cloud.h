// point_cloud.h
#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "kdtree.h"
namespace bpa {

// 前向声明
class KDTree;

class PointCloud {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 构造和析构函数
    PointCloud() = default;
    ~PointCloud() = default;

    // 点云数据
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
    
    // 计算包围盒
    struct BoundingBox {
        Eigen::Vector3f min;
        Eigen::Vector3f max;
        Eigen::Vector3f center;
        float diagonal;
    };

    // 基本操作
    void clear();
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    bool hasNormals() const { return !normals.empty(); }

    // 添加和访问点
    void addPoint(const Eigen::Vector3f& point);
    void addPointWithNormal(const Eigen::Vector3f& point, const Eigen::Vector3f& normal);
    const Eigen::Vector3f& getPoint(size_t index) const;
    const Eigen::Vector3f& getNormal(size_t index) const;

    // 计算法向量
    void computeNormals(int k_neighbors = 10);
    
    // 空间查询
    void buildKDTree();
    std::vector<size_t> findNeighbors(const Eigen::Vector3f& query, float radius) const;
    std::vector<size_t> findKNearestNeighbors(const Eigen::Vector3f& query, int k) const;

    // 点云变换
    void transform(const Eigen::Matrix4f& transformation);
    void translate(const Eigen::Vector3f& translation);
    void scale(float factor);
    
    // 点云处理
    void removeOutliers(int k_neighbors = 10, float std_dev_mult = 2.0f);
    void uniformDownsample(float grid_size);
    BoundingBox computeBoundingBox() const;

private:
    std::unique_ptr<KDTree> kdtree_;
    BoundingBox bounding_box_;

    // 内部辅助函数
    void updateBoundingBox();
    Eigen::Vector3f computeNormal(size_t point_index, const std::vector<size_t>& neighbors) const;
};

// 实现部分
inline void PointCloud::clear() {
    points.clear();
    normals.clear();
    kdtree_.reset();
}

inline void PointCloud::addPoint(const Eigen::Vector3f& point) {
    points.push_back(point);
    kdtree_.reset(); // 清除KD树缓存
}

inline void PointCloud::addPointWithNormal(const Eigen::Vector3f& point, const Eigen::Vector3f& normal) {
    points.push_back(point);
    normals.push_back(normal);
    kdtree_.reset();
}

inline const Eigen::Vector3f& PointCloud::getPoint(size_t index) const {
    return points[index];
}

inline const Eigen::Vector3f& PointCloud::getNormal(size_t index) const {
    return normals[index];
}

inline void PointCloud::transform(const Eigen::Matrix4f& transformation) {
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

    for (auto& point : points) {
        point = rotation * point + translation;
    }

    if (hasNormals()) {
        for (auto& normal : normals) {
            normal = rotation * normal;
            normal.normalize();
        }
    }

    kdtree_.reset();
    updateBoundingBox();
}

inline void PointCloud::translate(const Eigen::Vector3f& translation) {
    for (auto& point : points) {
        point += translation;
    }
    kdtree_.reset();
    updateBoundingBox();
}

inline void PointCloud::scale(float factor) {
    for (auto& point : points) {
        point *= factor;
    }
    kdtree_.reset();
    updateBoundingBox();
}

inline PointCloud::BoundingBox PointCloud::computeBoundingBox() const {
    BoundingBox bbox;
    if (points.empty()) {
        return bbox;
    }

    bbox.min = points[0];
    bbox.max = points[0];

    for (const auto& point : points) {
        bbox.min = bbox.min.cwiseMin(point);
        bbox.max = bbox.max.cwiseMax(point);
    }

    bbox.center = (bbox.max + bbox.min) * 0.5f;
    bbox.diagonal = (bbox.max - bbox.min).norm();
    return bbox;
}

inline void PointCloud::updateBoundingBox() {
    bounding_box_ = computeBoundingBox();
}

inline void PointCloud::computeNormals(int k_neighbors) {
    if (points.empty() || k_neighbors <= 0) {
        return;
    }

    // 确保KD树已建立
    if (!kdtree_) {
        buildKDTree();
    }

    normals.resize(points.size());

    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        // 查找近邻点
        auto neighbors = findKNearestNeighbors(points[i], k_neighbors);
        // 计算法向量
        normals[i] = computeNormal(i, neighbors);
    }
}

inline Eigen::Vector3f PointCloud::computeNormal(size_t point_index, const std::vector<size_t>& neighbors) const {
    // 计算协方差矩阵
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

    // 计算质心
    for (size_t idx : neighbors) {
        centroid += points[idx];
    }
    centroid /= static_cast<float>(neighbors.size());

    // 构建协方差矩阵
    for (size_t idx : neighbors) {
        Eigen::Vector3f diff = points[idx] - centroid;
        covariance += diff * diff.transpose();
    }

    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    
    // 最小特征值对应的特征向量即为法向量
    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);
    normal.normalize();
    
    return normal;
}
inline std::vector<size_t> PointCloud::findNeighbors(const Eigen::Vector3f& query, float radius) const {
    if (!kdtree_) {
        throw std::runtime_error("KD-tree not built. Call buildKDTree() first.");
    }
    return kdtree_->findNeighbors(query, radius);
}

inline std::vector<size_t> PointCloud::findKNearestNeighbors(const Eigen::Vector3f& query, int k) const {
    if (!kdtree_) {
        throw std::runtime_error("KD-tree not built. Call buildKDTree() first.");
    }
    if (k <= 0) {
        return std::vector<size_t>();
    }
    // 确保k不超过点云中的点数
    k = std::min(k, static_cast<int>(points.size()));
    return kdtree_->findKNearestNeighbors(query, k);
}
inline void PointCloud::buildKDTree() {
    if (points.empty()) {
        return;
    }
    kdtree_ = std::make_unique<KDTree>(points);
}
inline void PointCloud::removeOutliers(int k_neighbors, float std_dev_mult) {
    if (points.size() < k_neighbors + 1) {
        return;
    }

    // 确保KD树已建立
    if (!kdtree_) {
        buildKDTree();
    }

    std::vector<float> mean_distances(points.size());
    
    // 计算每个点到其邻近点的平均距离
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        auto neighbors = findKNearestNeighbors(points[i], k_neighbors);
        float sum_dist = 0.0f;
        
        for (size_t j : neighbors) {
            sum_dist += (points[i] - points[j]).norm();
        }
        
        mean_distances[i] = sum_dist / k_neighbors;
    }

    // 计算平均距离的均值和标准差
    float global_mean = 0.0f;
    for (float dist : mean_distances) {
        global_mean += dist;
    }
    global_mean /= mean_distances.size();

    float squared_sum = 0.0f;
    for (float dist : mean_distances) {
        float diff = dist - global_mean;
        squared_sum += diff * diff;
    }
    float std_dev = std::sqrt(squared_sum / mean_distances.size());

    // 移除离群点
    float threshold = global_mean + std_dev_mult * std_dev;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> filtered_points;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> filtered_normals;

    for (size_t i = 0; i < points.size(); ++i) {
        if (mean_distances[i] <= threshold) {
            filtered_points.push_back(points[i]);
            if (hasNormals()) {
                filtered_normals.push_back(normals[i]);
            }
        }
    }

    // 更新点云数据
    points = std::move(filtered_points);
    if (hasNormals()) {
        normals = std::move(filtered_normals);
    }

    // 重建KD树和更新包围盒
    kdtree_.reset();
    updateBoundingBox();
}

inline void PointCloud::uniformDownsample(float grid_size) {
    if (points.empty() || grid_size <= 0.0f) {
        return;
    }

    // 计算网格尺寸
    BoundingBox bbox = computeBoundingBox();
    Eigen::Vector3f grid_dimensions = (bbox.max - bbox.min) / grid_size;
    Eigen::Vector3i grid_size_3d(
        static_cast<int>(std::ceil(grid_dimensions.x())),
        static_cast<int>(std::ceil(grid_dimensions.y())),
        static_cast<int>(std::ceil(grid_dimensions.z()))
    );

    // 使用网格索引作为键的哈希表
    struct GridIndex {
        int x, y, z;
        
        bool operator==(const GridIndex& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
        
        struct Hash {
            size_t operator()(const GridIndex& index) const {
                return std::hash<int>()(index.x) ^
                       (std::hash<int>()(index.y) << 1) ^
                       (std::hash<int>()(index.z) << 2);
            }
        };
    };

    std::unordered_map<GridIndex, std::vector<size_t>, GridIndex::Hash> grid_map;

    // 将点分配到网格
    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3f relative_pos = (points[i] - bbox.min) / grid_size;
        GridIndex index {
            static_cast<int>(relative_pos.x()),
            static_cast<int>(relative_pos.y()),
            static_cast<int>(relative_pos.z())
        };
        grid_map[index].push_back(i);
    }

    // 对每个网格选择一个代表点
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> downsampled_points;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> downsampled_normals;

    for (const auto& grid_cell : grid_map) {
        const auto& indices = grid_cell.second;
        if (indices.empty()) continue;

        // 选择网格中心最近的点作为代表点
        Eigen::Vector3f grid_center = bbox.min + Eigen::Vector3f(
            (grid_cell.first.x + 0.5f) * grid_size,
            (grid_cell.first.y + 0.5f) * grid_size,
            (grid_cell.first.z + 0.5f) * grid_size
        );

        size_t closest_idx = indices[0];
        float min_dist = (points[indices[0]] - grid_center).squaredNorm();

        for (size_t i = 1; i < indices.size(); ++i) {
            float dist = (points[indices[i]] - grid_center).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = indices[i];
            }
        }

        downsampled_points.push_back(points[closest_idx]);
        if (hasNormals()) {
            downsampled_normals.push_back(normals[closest_idx]);
        }
    }

    // 更新点云数据
    points = std::move(downsampled_points);
    if (hasNormals()) {
        normals = std::move(downsampled_normals);
    }

    // 重建KD树和更新包围盒
    kdtree_.reset();
    updateBoundingBox();
}
} // namespace bpa