// bpa.h
#pragma once
#include <set>
#include <unordered_map>
#include <list>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Labs/2-GeometryProcessing/mesh.h"
#include "Labs/2-GeometryProcessing/kdtree.h"
#include "Labs/2-GeometryProcessing/io.h"

namespace bpa {

class BallPivoting {
private:
    // 边结构，用于跟踪活动边
    struct Edge {
        size_t v1, v2;
        float quality;  // 边的质量评分

        Edge(size_t _v1, size_t _v2, float _quality = 0.0f) 
            : v1(_v1), v2(_v2), quality(_quality) {}

        bool operator==(const Edge& other) const {
            return (v1 == other.v1 && v2 == other.v2) ||
                   (v1 == other.v2 && v2 == other.v1);
        }

        bool operator<(const Edge& other) const {
            if (quality != other.quality)
                return quality > other.quality;  // 高质量优先
            auto minmax1 = std::minmax(v1, v2);
            auto minmax2 = std::minmax(other.v1, other.v2);
            if (minmax1.first != minmax2.first)
                return minmax1.first < minmax2.first;
            return minmax1.second < minmax2.second;
        }
    };

    struct Triangle {
        size_t v1, v2, v3;
        float quality;  // 三角形质量评分

        Triangle(size_t _v1, size_t _v2, size_t _v3, float _quality = 0.0f)
            : v1(_v1), v2(_v2), v3(_v3), quality(_quality) {}
    };

    const PointCloud& cloud_;
    float radius_;
    std::vector<Triangle> triangles_;
    std::set<Edge> active_edges_;
    std::set<size_t> used_vertices_;
    float min_edge_length_;  // 最小边长
    float max_edge_length_;  // 最大边长
    
    // 新增：用于跟踪重建质量的参数
    float max_angle_deviation_;  // 法线角度偏差阈值
    float min_triangle_quality_; // 最小三角形质量阈值

public:
    BallPivoting(const PointCloud& cloud, float radius)
        : cloud_(cloud), radius_(radius) {
        if (!cloud.hasNormals()) {
            throw std::runtime_error("Point cloud must have normals for BPA");
        }
        
        // 初始化参数
        min_edge_length_ = radius_ * 0.1f;
        max_edge_length_ = radius_ * 2.0f;
        max_angle_deviation_ = 0.7f;  // 约45度
        min_triangle_quality_ = 0.3f;
        
        std::cout << "BPA Parameters:" << std::endl
                 << "- Radius: " << radius_ << std::endl
                 << "- Min edge length: " << min_edge_length_ << std::endl
                 << "- Max edge length: " << max_edge_length_ << std::endl;
    }

    const PointCloud& getCloud() const { return cloud_; }
    bool reconstruct();
    const std::vector<Triangle>& getTriangles() const { return triangles_; }

private:
    // 核心几何计算函数
    Eigen::Vector3f computeBallCenter(const Eigen::Vector3f& p1,
                                    const Eigen::Vector3f& p2,
                                    const Eigen::Vector3f& p3) const {
        // 计算三角形的外接球心
        Eigen::Vector3f edge1 = p2 - p1;
        Eigen::Vector3f edge2 = p3 - p1;
        Eigen::Vector3f normal = edge1.cross(edge2).normalized();
        
        // 构建线性方程组求解球心
        Eigen::Matrix3f A;
        A << 2*edge1.dot(edge1), 2*edge1.dot(edge2), edge1.dot(normal),
             2*edge1.dot(edge2), 2*edge2.dot(edge2), edge2.dot(normal),
             edge1.dot(normal), edge2.dot(normal), 0;
        
        Eigen::Vector3f b(0, 0, radius_ * radius_);
        Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
        
        return p1 + x[0]*edge1 + x[1]*edge2 + x[2]*normal;
    }

    float computeTriangleQuality(size_t v1, size_t v2, size_t v3) const {
        const auto& p1 = cloud_.getPoint(v1);
        const auto& p2 = cloud_.getPoint(v2);
        const auto& p3 = cloud_.getPoint(v3);
        
        // 计算边长
        float d12 = (p2 - p1).norm();
        float d23 = (p3 - p2).norm();
        float d31 = (p1 - p3).norm();
        
        // 计算最小和最大边长比
        float min_edge = std::min({d12, d23, d31});
        float max_edge = std::max({d12, d23, d31});
        float edge_ratio = min_edge / max_edge;
        
        // 计算法线一致性
        const auto& n1 = cloud_.getNormal(v1);
        const auto& n2 = cloud_.getNormal(v2);
        const auto& n3 = cloud_.getNormal(v3);
        float normal_consistency = std::min({
            n1.dot(n2),
            n2.dot(n3),
            n3.dot(n1)
        });
        
        // 综合评分
        return edge_ratio * (normal_consistency + 1.0f) * 0.5f;
    }

    std::vector<size_t> findSeedTriangle() {
        std::cout << "Finding seed triangle..." << std::endl;
        float best_quality = -1.0f;
        std::vector<size_t> best_seed;
        
        // 采样点寻找最佳种子三角形
        for (size_t i = 0; i < cloud_.size(); i += cloud_.size()/100) {
            if (used_vertices_.find(i) != used_vertices_.end())
                continue;

            const auto& p1 = cloud_.getPoint(i);
            const auto& n1 = cloud_.getNormal(i);
            
            // 寻找邻域点
            std::vector<size_t> neighbors = cloud_.findNeighbors(p1, radius_ * 2.0f);
            if (neighbors.size() < 2) continue;
            
            // 在邻域中寻找最佳三角形
            for (size_t j = 0; j < neighbors.size(); ++j) {
                for (size_t k = j + 1; k < neighbors.size(); ++k) {
                    size_t v2 = neighbors[j];
                    size_t v3 = neighbors[k];
                    
                    if (!isValidTriangle(i, v2, v3)) continue;
                    
                    float quality = computeTriangleQuality(i, v2, v3);
                    if (quality > best_quality) {
                        best_quality = quality;
                        best_seed = {i, v2, v3};
                    }
                }
            }
        }
        
        if (!best_seed.empty()) {
            std::cout << "Found seed triangle with quality: " << best_quality << std::endl;
        }
        return best_seed;
    }

    bool isValidTriangle(size_t v1, size_t v2, size_t v3) const {
        
        if (v1 == v2 || v2 == v3 || v3 == v1) {
            return false;
        }
        
        const auto& p1 = cloud_.getPoint(v1);
        const auto& p2 = cloud_.getPoint(v2);
        const auto& p3 = cloud_.getPoint(v3);
        
        // 检查边长
        float d12 = (p2 - p1).norm();
        float d23 = (p3 - p2).norm();
        float d31 = (p1 - p3).norm();
        
        if (d12 < min_edge_length_ || d23 < min_edge_length_ || d31 < min_edge_length_ ||
            d12 > max_edge_length_ || d23 > max_edge_length_ || d31 > max_edge_length_)
            return false;
        return 1;
        // 检查法线一致性
        const auto& n1 = cloud_.getNormal(v1);
        const auto& n2 = cloud_.getNormal(v2);
        const auto& n3 = cloud_.getNormal(v3);
        
        if (n1.dot(n2) < max_angle_deviation_ || 
            n2.dot(n3) < max_angle_deviation_ || 
            n3.dot(n1) < max_angle_deviation_)
            return false;
            
        return true;
    }

    bool pivotBall(const Edge& edge, size_t& next_vertex) {
        const auto& p1 = cloud_.getPoint(edge.v1);
        const auto& p2 = cloud_.getPoint(edge.v2);
        Eigen::Vector3f midpoint = (p1 + p2) * 0.5f;
        
        // 搜索候选点
        std::vector<size_t> neighbors = cloud_.findNeighbors(midpoint, radius_ * 3.0f);
        
        float best_quality = -1.0f;
        bool found = false;
        
        Eigen::Vector3f edge_dir = (p2 - p1).normalized();
        Eigen::Vector3f edge_normal = (cloud_.getNormal(edge.v1) + 
                                     cloud_.getNormal(edge.v2)).normalized();
        
        for (size_t candidate : neighbors) {
            if (candidate == edge.v1 || candidate == edge.v2 ||
                used_vertices_.find(candidate) != used_vertices_.end())
                continue;
            
            if (!isValidTriangle(edge.v1, edge.v2, candidate))
                continue;
            
            const auto& p3 = cloud_.getPoint(candidate);
            Eigen::Vector3f center = computeBallCenter(p1, p2, p3);
            
            // 检查球面上是否有其他点
            bool is_empty = true;
            for (size_t idx : neighbors) {
                if (idx == edge.v1 || idx == edge.v2 || idx == candidate)
                    continue;
                    
                if ((cloud_.getPoint(idx) - center).norm() < radius_ * 0.99f) {
                    is_empty = false;
                    break;
                }
            }
            
            if (!is_empty) continue;
            
            // 计算三角形质量
            float quality = computeTriangleQuality(edge.v1, edge.v2, candidate);
            if (quality > best_quality) {
                best_quality = quality;
                next_vertex = candidate;
                found = true;
            }
        }
        
        return found && best_quality >= min_triangle_quality_;
    }
};

inline bool BallPivoting::reconstruct() {
    std::cout << "Starting reconstruction..." << std::endl;
    std::cout << "Point cloud size: " << cloud_.size() << std::endl;
    
    triangles_.clear();
    active_edges_.clear();
    used_vertices_.clear();

    // 找到种子三角形
    auto seed_vertices = findSeedTriangle();
    if (seed_vertices.empty()) {
        std::cerr << "Failed to find seed triangle" << std::endl;
        return false;
    }

    // 添加第一个三角形
    float quality = computeTriangleQuality(
        seed_vertices[0], seed_vertices[1], seed_vertices[2]);
    triangles_.emplace_back(
        seed_vertices[0], seed_vertices[1], seed_vertices[2], quality);
        
    for (size_t v : seed_vertices) {
        used_vertices_.insert(v);
    }

    // 添加初始活动边
    active_edges_.insert(Edge(seed_vertices[0], seed_vertices[1], quality));
    active_edges_.insert(Edge(seed_vertices[1], seed_vertices[2], quality));
    active_edges_.insert(Edge(seed_vertices[2], seed_vertices[0], quality));

    size_t iteration = 0;
    size_t max_iterations = cloud_.size() * 10;  // 防止无限循环

    while (!active_edges_.empty() && iteration++ < max_iterations) {
        // 取出当前最高质量的边
        Edge current_edge = *active_edges_.begin();
        active_edges_.erase(active_edges_.begin());

        size_t next_vertex;
        if (pivotBall(current_edge, next_vertex)) {
            float quality = computeTriangleQuality(
                current_edge.v1, current_edge.v2, next_vertex);
                
            // 添加新三角形
            triangles_.emplace_back(
                current_edge.v1, current_edge.v2, next_vertex, quality);
            used_vertices_.insert(next_vertex);

            // 处理新边
            Edge e1(current_edge.v1, next_vertex, quality);
            Edge e2(current_edge.v2, next_vertex, quality);

            if (active_edges_.find(e1) != active_edges_.end())
                active_edges_.erase(e1);
            else
                active_edges_.insert(e1);

            if (active_edges_.find(e2) != active_edges_.end())
                active_edges_.erase(e2);
            else
                active_edges_.insert(e2);
        }
        
        if (iteration % 1000 == 0) {
            std::cout << "Iteration " << iteration << ": "
                     << triangles_.size() << " triangles, "
                     << active_edges_.size() << " active edges" << std::endl;
        }
    }

    std::cout << "Reconstruction completed:" << std::endl;
    return 1;
}

             

// 用于将重建结果转换为表面网格的辅助函数
inline void convertToMesh(const BallPivoting& bpa, VCX::Engine::SurfaceMesh& mesh) {
    std::cout << "\n=== Starting Mesh Conversion ===" << std::endl;
    mesh.Positions.clear();
    mesh.Normals.clear();
    mesh.TexCoords.clear();
    mesh.Indices.clear();
    
    // 获取三角形列表
    const auto& triangles = bpa.getTriangles();
    const auto& cloud = bpa.getCloud(); // 获取点云数据
    std::cout << "Input data statistics:" << std::endl;
    std::cout << "- Point cloud size: " << cloud.size() << std::endl;
    std::cout << "- Number of triangles: " << triangles.size() << std::endl;
    std::cout << "- Has normals: " << (cloud.hasNormals() ? "yes" : "no") << std::endl;
    // 首先添加所有顶点数据
    // mesh.Positions.push_back(glm::vec3(1,0,1));
    // mesh.Positions.push_back(glm::vec3(1,0,0));
    // mesh.Positions.push_back(glm::vec3(0,0,1));
    for (size_t i = 0; i < cloud.size(); ++i) {
        // 转换顶点位置 (Eigen::Vector3f 到 glm::vec3)
        const auto& point = cloud.getPoint(i);
        mesh.Positions.push_back(glm::vec3(point.x(), point.y(), point.z()));
        
        
        // 转换法线数据 (如果点云有法线)
        // if (cloud.hasNormals()) {
        //     const auto& normal = cloud.getNormal(i);
        //     mesh.Normals.push_back(glm::vec3(normal.x(), normal.y(), normal.z()));
        // }
        
    }
    
    // 添加三角形索引
    for (const auto& tri : triangles) {
        mesh.Indices.push_back(static_cast<std::uint32_t>(tri.v1));
        mesh.Indices.push_back(static_cast<std::uint32_t>(tri.v2));
        mesh.Indices.push_back(static_cast<std::uint32_t>(tri.v3));
        mesh.Indices.push_back(static_cast<std::uint32_t>(tri.v3));
        mesh.Indices.push_back(static_cast<std::uint32_t>(tri.v2));
        mesh.Indices.push_back(static_cast<std::uint32_t>(tri.v1));
    }
    // mesh.Indices.push_back(0);
    // mesh.Indices.push_back(1);
    // mesh.Indices.push_back(2);
    // mesh.Indices.push_back(2);
    // mesh.Indices.push_back(1);
    // mesh.Indices.push_back(0);
    // 如果点云没有法线，计算法线
    // if (!cloud.hasNormals()) {
    //     mesh.Normals = mesh.ComputeNormals();
    // }
}


} // namespace bpa