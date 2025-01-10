// point_cloud.h
#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

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

} // namespace bpa