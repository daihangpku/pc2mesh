#pragma once

#include "Labs/2-GeometryProcessing/ModelObject.h"
#include <vector>
#include <memory>
#include <queue>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <numeric>  
#include <Eigen/Dense>
// namespace bpa{

// class KDTree{
// public:
//     struct Node {
//         Eigen::Vector3f point;
//         size_t index;      // 原始点云中的索引
//         int axis;          // 分割轴
//         std::unique_ptr<Node> left;
//         std::unique_ptr<Node> right;
        
//         Node(const Eigen::Vector3f& p, size_t idx): point(p), index(idx), axis(0), left(nullptr), right(nullptr) {}
//     };
//     struct NeighborCandidate {
//         size_t index;
//         float distance;
        
//         NeighborCandidate(size_t idx, float dist) 
//             : index(idx), distance(dist) {}
        
//         bool operator>(const NeighborCandidate& other) const {
//             return distance > other.distance;
//         }
//     };
//     // 构造函数
//     KDTree(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points) {
//         buildTree(points);
//     }
//     // 查找最近邻
//     std::vector<size_t> findNeighbors(const Eigen::Vector3f& query, float radius) const;
//     std::vector<size_t> findKNearestNeighbors(const Eigen::Vector3f& query, int k) const;
// private:
//     std::unique_ptr<Node> root_;
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > points_;
//     // 构建树
//     void buildTree(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points){
//         points_ = points;
//         std::vector<size_t> indices(points.size());
//         std::iota(indices.begin(), indices.end(), 0);
//         root_ = buildTreeRecursive(indices, 0);
//     }
//     std::unique_ptr<Node> buildTreeRecursive(std::vector<size_t>& indices, int depth);
//     // 查找辅助函数
//     void searchRadius(const Node* node, const Eigen::Vector3f& query, float radius,
//                      std::vector<size_t>& results) const;
                     
//     void searchKNN(const Node* node, const Eigen::Vector3f& query, int k,
//                   std::priority_queue<NeighborCandidate, 
//                   std::vector<NeighborCandidate>,
//                   std::greater<NeighborCandidate>>& pq) const;
//     // 计算点到分割平面的距离
//     float distanceToSplitPlane(const Node* node, const Eigen::Vector3f& query) const {
//         return std::abs(query[node->axis] - node->point[node->axis]);
//     }

//     // 计算两点之间的欧氏距离平方
//     float distanceSquared(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) const {
//         return (p1 - p2).squaredNorm();
//     }
// };

// inline std::unique_ptr<KDTree::Node> KDTree::buildTreeRecursive(
//     std::vector<size_t>& indices, int depth) {
//     if (indices.empty()) {
//         return nullptr;
//     }

//     // 选择分割轴 (通过深度循环选择 x, y, z)
//     int axis = depth % 3;

//     // 按选定轴排序并找到中位数
//     std::sort(indices.begin(), indices.end(),
//         [&](size_t a, size_t b) {
//             return points_[a][axis] < points_[b][axis];
//         });

//     size_t median_idx = indices.size() / 2;
//     size_t point_idx = indices[median_idx];

//     // 创建节点
//     auto node = std::make_unique<Node>(points_[point_idx], point_idx);
//     node->axis = axis;

//     // 递归构建左右子树
//     std::vector<size_t> left_indices(indices.begin(), indices.begin() + median_idx);
//     std::vector<size_t> right_indices(indices.begin() + median_idx + 1, indices.end());

//     node->left = buildTreeRecursive(left_indices, depth + 1);
//     node->right = buildTreeRecursive(right_indices, depth + 1);

//     return node;
// }
// inline std::vector<size_t> KDTree::findNeighbors(const Eigen::Vector3f& query, float radius) const {
//     std::vector<size_t> results;
//     float squared_radius = radius * radius;
//     searchRadius(root_.get(), query, squared_radius, results);
//     return results;
// }
// inline void KDTree::searchRadius(const Node* node, const Eigen::Vector3f& query, float squared_radius, std::vector<size_t>& results) const {
    
//     if (!node) return;

//     // 检查当前点是否在范围内
//     float squared_dist = distanceSquared(query, node->point);
//     if (squared_dist <= squared_radius) {
//         results.push_back(node->index);
//     }

//     // 计算到分割平面的距离
//     float dist_to_plane = distanceToSplitPlane(node, query);
//     bool go_left = query[node->axis] < node->point[node->axis];

//     // 递归搜索最可能包含结果的子树
//     searchRadius(go_left ? node->left.get() : node->right.get(),
//                 query, squared_radius, results);

//     // 如果到分割平面的距离小于搜索半径，也搜索另一个子树
//     if (dist_to_plane * dist_to_plane <= squared_radius) {
//         searchRadius(go_left ? node->right.get() : node->left.get(),
//                     query, squared_radius, results);
//     }
// }
// inline std::vector<size_t> KDTree::findKNearestNeighbors(
//     const Eigen::Vector3f& query, int k) const {
    
//     std::priority_queue<NeighborCandidate,
//                        std::vector<NeighborCandidate>,
//                        std::greater<NeighborCandidate>> pq;
    
//     searchKNN(root_.get(), query, k, pq);

//     std::vector<size_t> results;
//     results.reserve(pq.size());
    
//     while (!pq.empty()) {
//         results.push_back(pq.top().index);
//         pq.pop();
//     }
    
//     std::reverse(results.begin(), results.end());
//     return results;
// }
// inline void KDTree::searchKNN(
//     const Node* node, const Eigen::Vector3f& query, int k,
//     std::priority_queue<NeighborCandidate,
//                        std::vector<NeighborCandidate>,
//                        std::greater<NeighborCandidate>>& pq) const {
    
//     if (!node) return;

//     // 计算到当前点的距离
//     float squared_dist = distanceSquared(query, node->point);
    
//     // 如果还没找到k个点，或者当前点比最远的点更近，则加入队列
//     if (pq.size() < static_cast<size_t>(k)) {
//         pq.emplace(node->index, squared_dist);
//     } else if (squared_dist < pq.top().distance) {
//         pq.pop();
//         pq.emplace(node->index, squared_dist);
//     }

//     // 确定搜索顺序
//     float dist_to_plane = distanceToSplitPlane(node, query);
//     bool go_left = query[node->axis] < node->point[node->axis];

//     // 递归搜索最可能的子树
//     searchKNN(go_left ? node->left.get() : node->right.get(),
//              query, k, pq);

//     // 检查是否需要搜索另一个子树
//     float worst_dist = pq.size() < static_cast<size_t>(k) ?
//                       std::numeric_limits<float>::max() :
//                       pq.top().distance;
                      
//     if (dist_to_plane * dist_to_plane <= worst_dist) {
//         searchKNN(go_left ? node->right.get() : node->left.get(),
//                  query, k, pq);
//     }
// }

// // point_cloud.h







// class PointCloud {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     // 构造和析构函数
//     PointCloud() = default;
//     ~PointCloud() = default;

//     // 点云数据
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points;
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
    
//     // 计算包围盒
//     struct BoundingBox {
//         Eigen::Vector3f min;
//         Eigen::Vector3f max;
//         Eigen::Vector3f center;
//         float diagonal;
//     };

//     // 基本操作
//     void clear();
//     size_t size() const { return points.size(); }
//     bool empty() const { return points.empty(); }
//     bool hasNormals() const { return !normals.empty(); }

//     // 添加和访问点
//     void addPoint(const Eigen::Vector3f& point);
//     void addPointWithNormal(const Eigen::Vector3f& point, const Eigen::Vector3f& normal);
//     const Eigen::Vector3f& getPoint(size_t index) const;
//     const Eigen::Vector3f& getNormal(size_t index) const;

//     // 计算法向量
//     void computeNormals(int k_neighbors = 10);
    
//     // 空间查询
//     void buildKDTree();
//     std::vector<size_t> findNeighbors(const Eigen::Vector3f& query, float radius) const;
//     std::vector<size_t> findKNearestNeighbors(const Eigen::Vector3f& query, int k) const;

//     // 点云变换
//     void transform(const Eigen::Matrix4f& transformation);
//     void translate(const Eigen::Vector3f& translation);
//     void scale(float factor);
    
//     // 点云处理
//     void removeOutliers(int k_neighbors = 10, float std_dev_mult = 2.0f);
//     void uniformDownsample(float grid_size);
//     BoundingBox computeBoundingBox() const;

// private:
//     std::unique_ptr<KDTree> kdtree_;
//     BoundingBox bounding_box_;

//     // 内部辅助函数
//     void updateBoundingBox();
//     Eigen::Vector3f computeNormal(size_t point_index, const std::vector<size_t>& neighbors) const;
// };

// // 实现部分
// inline void PointCloud::clear() {
//     points.clear();
//     normals.clear();
//     kdtree_.reset();
// }

// inline void PointCloud::addPoint(const Eigen::Vector3f& point) {
//     points.push_back(point);
//     kdtree_.reset(); // 清除KD树缓存
// }

// inline void PointCloud::addPointWithNormal(const Eigen::Vector3f& point, const Eigen::Vector3f& normal) {
//     points.push_back(point);
//     normals.push_back(normal);
//     kdtree_.reset();
// }

// inline const Eigen::Vector3f& PointCloud::getPoint(size_t index) const {
//     return points[index];
// }

// inline const Eigen::Vector3f& PointCloud::getNormal(size_t index) const {
//     return normals[index];
// }

// inline void PointCloud::transform(const Eigen::Matrix4f& transformation) {
//     Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
//     Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

//     for (auto& point : points) {
//         point = rotation * point + translation;
//     }

//     if (hasNormals()) {
//         for (auto& normal : normals) {
//             normal = rotation * normal;
//             normal.normalize();
//         }
//     }

//     kdtree_.reset();
//     updateBoundingBox();
// }

// inline void PointCloud::translate(const Eigen::Vector3f& translation) {
//     for (auto& point : points) {
//         point += translation;
//     }
//     kdtree_.reset();
//     updateBoundingBox();
// }

// inline void PointCloud::scale(float factor) {
//     for (auto& point : points) {
//         point *= factor;
//     }
//     kdtree_.reset();
//     updateBoundingBox();
// }

// inline PointCloud::BoundingBox PointCloud::computeBoundingBox() const {
//     BoundingBox bbox;
//     if (points.empty()) {
//         return bbox;
//     }

//     bbox.min = points[0];
//     bbox.max = points[0];

//     for (const auto& point : points) {
//         bbox.min = bbox.min.cwiseMin(point);
//         bbox.max = bbox.max.cwiseMax(point);
//     }

//     bbox.center = (bbox.max + bbox.min) * 0.5f;
//     bbox.diagonal = (bbox.max - bbox.min).norm();
//     return bbox;
// }

// inline void PointCloud::updateBoundingBox() {
//     bounding_box_ = computeBoundingBox();
// }

// inline void PointCloud::computeNormals(int k_neighbors) {
//     if (points.empty() || k_neighbors <= 0) {
//         return;
//     }

//     // 确保KD树已建立
//     if (!kdtree_) {
//         buildKDTree();
//     }

//     normals.resize(points.size());

//     #pragma omp parallel for
//     for (int i = 0; i < static_cast<int>(points.size()); ++i) {
//         // 查找近邻点
//         auto neighbors = findKNearestNeighbors(points[i], k_neighbors);
//         // 计算法向量
//         normals[i] = computeNormal(i, neighbors);
//     }
// }

// inline Eigen::Vector3f PointCloud::computeNormal(size_t point_index, const std::vector<size_t>& neighbors) const {
//     // 计算协方差矩阵
//     Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
//     Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

//     // 计算质心
//     for (size_t idx : neighbors) {
//         centroid += points[idx];
//     }
//     centroid /= static_cast<float>(neighbors.size());

//     // 构建协方差矩阵
//     for (size_t idx : neighbors) {
//         Eigen::Vector3f diff = points[idx] - centroid;
//         covariance += diff * diff.transpose();
//     }

//     // 特征值分解
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    
//     // 最小特征值对应的特征向量即为法向量
//     Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);
//     normal.normalize();
    
//     return normal;
// }
// inline std::vector<size_t> PointCloud::findNeighbors(const Eigen::Vector3f& query, float radius) const {
//     if (!kdtree_) {
//         throw std::runtime_error("KD-tree not built. Call buildKDTree() first.");
//     }
//     return kdtree_->findNeighbors(query, radius);
// }

// inline std::vector<size_t> PointCloud::findKNearestNeighbors(const Eigen::Vector3f& query, int k) const {
//     if (!kdtree_) {
//         throw std::runtime_error("KD-tree not built. Call buildKDTree() first.");
//     }
//     if (k <= 0) {
//         return std::vector<size_t>();
//     }
//     // 确保k不超过点云中的点数
//     k = std::min(k, static_cast<int>(points.size()));
//     return kdtree_->findKNearestNeighbors(query, k);
// }
// inline void PointCloud::buildKDTree() {
//     if (points.empty()) {
//         return;
//     }
//     kdtree_ = std::make_unique<KDTree>(points);
// }
// inline void PointCloud::removeOutliers(int k_neighbors, float std_dev_mult) {
//     if (points.size() < k_neighbors + 1) {
//         return;
//     }

//     // 确保KD树已建立
//     if (!kdtree_) {
//         buildKDTree();
//     }

//     std::vector<float> mean_distances(points.size());
    
//     // 计算每个点到其邻近点的平均距离
//     #pragma omp parallel for
//     for (int i = 0; i < static_cast<int>(points.size()); ++i) {
//         auto neighbors = findKNearestNeighbors(points[i], k_neighbors);
//         float sum_dist = 0.0f;
        
//         for (size_t j : neighbors) {
//             sum_dist += (points[i] - points[j]).norm();
//         }
        
//         mean_distances[i] = sum_dist / k_neighbors;
//     }

//     // 计算平均距离的均值和标准差
//     float global_mean = 0.0f;
//     for (float dist : mean_distances) {
//         global_mean += dist;
//     }
//     global_mean /= mean_distances.size();

//     float squared_sum = 0.0f;
//     for (float dist : mean_distances) {
//         float diff = dist - global_mean;
//         squared_sum += diff * diff;
//     }
//     float std_dev = std::sqrt(squared_sum / mean_distances.size());

//     // 移除离群点
//     float threshold = global_mean + std_dev_mult * std_dev;
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> filtered_points;
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> filtered_normals;

//     for (size_t i = 0; i < points.size(); ++i) {
//         if (mean_distances[i] <= threshold) {
//             filtered_points.push_back(points[i]);
//             if (hasNormals()) {
//                 filtered_normals.push_back(normals[i]);
//             }
//         }
//     }

//     // 更新点云数据
//     points = std::move(filtered_points);
//     if (hasNormals()) {
//         normals = std::move(filtered_normals);
//     }

//     // 重建KD树和更新包围盒
//     kdtree_.reset();
//     updateBoundingBox();
// }

// inline void PointCloud::uniformDownsample(float grid_size) {
//     if (points.empty() || grid_size <= 0.0f) {
//         return;
//     }

//     // 计算网格尺寸
//     BoundingBox bbox = computeBoundingBox();
//     Eigen::Vector3f grid_dimensions = (bbox.max - bbox.min) / grid_size;
//     Eigen::Vector3i grid_size_3d(
//         static_cast<int>(std::ceil(grid_dimensions.x())),
//         static_cast<int>(std::ceil(grid_dimensions.y())),
//         static_cast<int>(std::ceil(grid_dimensions.z()))
//     );

//     // 使用网格索引作为键的哈希表
//     struct GridIndex {
//         int x, y, z;
        
//         bool operator==(const GridIndex& other) const {
//             return x == other.x && y == other.y && z == other.z;
//         }
        
//         struct Hash {
//             size_t operator()(const GridIndex& index) const {
//                 return std::hash<int>()(index.x) ^
//                        (std::hash<int>()(index.y) << 1) ^
//                        (std::hash<int>()(index.z) << 2);
//             }
//         };
//     };

//     std::unordered_map<GridIndex, std::vector<size_t>, GridIndex::Hash> grid_map;

//     // 将点分配到网格
//     for (size_t i = 0; i < points.size(); ++i) {
//         Eigen::Vector3f relative_pos = (points[i] - bbox.min) / grid_size;
//         GridIndex index {
//             static_cast<int>(relative_pos.x()),
//             static_cast<int>(relative_pos.y()),
//             static_cast<int>(relative_pos.z())
//         };
//         grid_map[index].push_back(i);
//     }

//     // 对每个网格选择一个代表点
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> downsampled_points;
//     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> downsampled_normals;

//     for (const auto& grid_cell : grid_map) {
//         const auto& indices = grid_cell.second;
//         if (indices.empty()) continue;

//         // 选择网格中心最近的点作为代表点
//         Eigen::Vector3f grid_center = bbox.min + Eigen::Vector3f(
//             (grid_cell.first.x + 0.5f) * grid_size,
//             (grid_cell.first.y + 0.5f) * grid_size,
//             (grid_cell.first.z + 0.5f) * grid_size
//         );

//         size_t closest_idx = indices[0];
//         float min_dist = (points[indices[0]] - grid_center).squaredNorm();

//         for (size_t i = 1; i < indices.size(); ++i) {
//             float dist = (points[indices[i]] - grid_center).squaredNorm();
//             if (dist < min_dist) {
//                 min_dist = dist;
//                 closest_idx = indices[i];
//             }
//         }

//         downsampled_points.push_back(points[closest_idx]);
//         if (hasNormals()) {
//             downsampled_normals.push_back(normals[closest_idx]);
//         }
//     }

//     // 更新点云数据
//     points = std::move(downsampled_points);
//     if (hasNormals()) {
//         normals = std::move(downsampled_normals);
//     }

//     // 重建KD树和更新包围盒
//     kdtree_.reset();
//     updateBoundingBox();
// }

// }//namespace bpa
namespace VCX::Labs::GeometryProcessing {

    
    void pc2mesh(Engine::SurfaceMesh& output, 
                   const std::string& data_path, 
                   const double r);

}
