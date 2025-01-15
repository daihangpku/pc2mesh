//kptree.h
#pragma once

#include <vector>
#include <memory>
#include <queue>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <numeric>  
namespace bpa{

class KDTree{
public:
    struct Node {
        Eigen::Vector3f point;
        size_t index;      // 原始点云中的索引
        int axis;          // 分割轴
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
        
        Node(const Eigen::Vector3f& p, size_t idx): point(p), index(idx), axis(0), left(nullptr), right(nullptr) {}
    };
    struct NeighborCandidate {
        size_t index;
        float distance;
        
        NeighborCandidate(size_t idx, float dist) 
            : index(idx), distance(dist) {}
        
        bool operator>(const NeighborCandidate& other) const {
            return distance > other.distance;
        }
    };
    // 构造函数
    KDTree(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points) {
        buildTree(points);
    }
    // 查找最近邻
    std::vector<size_t> findNeighbors(const Eigen::Vector3f& query, float radius) const;
    std::vector<size_t> findKNearestNeighbors(const Eigen::Vector3f& query, int k) const;
private:
    std::unique_ptr<Node> root_;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > points_;
    // 构建树
    void buildTree(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points){
        points_ = points;
        std::vector<size_t> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        root_ = buildTreeRecursive(indices, 0);
    }
    std::unique_ptr<Node> buildTreeRecursive(std::vector<size_t>& indices, int depth);
    // 查找辅助函数
    void searchRadius(const Node* node, const Eigen::Vector3f& query, float radius,
                     std::vector<size_t>& results) const;
                     
    void searchKNN(const Node* node, const Eigen::Vector3f& query, int k,
                  std::priority_queue<NeighborCandidate, 
                  std::vector<NeighborCandidate>,
                  std::greater<NeighborCandidate>>& pq) const;
    // 计算点到分割平面的距离
    float distanceToSplitPlane(const Node* node, const Eigen::Vector3f& query) const {
        return std::abs(query[node->axis] - node->point[node->axis]);
    }

    // 计算两点之间的欧氏距离平方
    float distanceSquared(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) const {
        return (p1 - p2).squaredNorm();
    }
};

inline std::unique_ptr<KDTree::Node> KDTree::buildTreeRecursive(
    std::vector<size_t>& indices, int depth) {
    if (indices.empty()) {
        return nullptr;
    }

    // 选择分割轴 (通过深度循环选择 x, y, z)
    int axis = depth % 3;

    // 按选定轴排序并找到中位数
    std::sort(indices.begin(), indices.end(),
        [&](size_t a, size_t b) {
            return points_[a][axis] < points_[b][axis];
        });

    size_t median_idx = indices.size() / 2;
    size_t point_idx = indices[median_idx];

    // 创建节点
    auto node = std::make_unique<Node>(points_[point_idx], point_idx);
    node->axis = axis;

    // 递归构建左右子树
    std::vector<size_t> left_indices(indices.begin(), indices.begin() + median_idx);
    std::vector<size_t> right_indices(indices.begin() + median_idx + 1, indices.end());

    node->left = buildTreeRecursive(left_indices, depth + 1);
    node->right = buildTreeRecursive(right_indices, depth + 1);

    return node;
}
inline std::vector<size_t> KDTree::findNeighbors(const Eigen::Vector3f& query, float radius) const {
    std::vector<size_t> results;
    float squared_radius = radius * radius;
    searchRadius(root_.get(), query, squared_radius, results);
    return results;
}
inline void KDTree::searchRadius(const Node* node, const Eigen::Vector3f& query, float squared_radius, std::vector<size_t>& results) const {
    
    if (!node) return;

    // 检查当前点是否在范围内
    float squared_dist = distanceSquared(query, node->point);
    if (squared_dist <= squared_radius) {
        results.push_back(node->index);
    }

    // 计算到分割平面的距离
    float dist_to_plane = distanceToSplitPlane(node, query);
    bool go_left = query[node->axis] < node->point[node->axis];

    // 递归搜索最可能包含结果的子树
    searchRadius(go_left ? node->left.get() : node->right.get(),
                query, squared_radius, results);

    // 如果到分割平面的距离小于搜索半径，也搜索另一个子树
    if (dist_to_plane * dist_to_plane <= squared_radius) {
        searchRadius(go_left ? node->right.get() : node->left.get(),
                    query, squared_radius, results);
    }
}
inline std::vector<size_t> KDTree::findKNearestNeighbors(
    const Eigen::Vector3f& query, int k) const {
    
    std::priority_queue<NeighborCandidate,
                       std::vector<NeighborCandidate>,
                       std::greater<NeighborCandidate>> pq;
    
    searchKNN(root_.get(), query, k, pq);

    std::vector<size_t> results;
    results.reserve(pq.size());
    
    while (!pq.empty()) {
        results.push_back(pq.top().index);
        pq.pop();
    }
    
    std::reverse(results.begin(), results.end());
    return results;
}
inline void KDTree::searchKNN(
    const Node* node, const Eigen::Vector3f& query, int k,
    std::priority_queue<NeighborCandidate,
                       std::vector<NeighborCandidate>,
                       std::greater<NeighborCandidate>>& pq) const {
    
    if (!node) return;

    // 计算到当前点的距离
    float squared_dist = distanceSquared(query, node->point);
    
    // 如果还没找到k个点，或者当前点比最远的点更近，则加入队列
    if (pq.size() < static_cast<size_t>(k)) {
        pq.emplace(node->index, squared_dist);
    } else if (squared_dist < pq.top().distance) {
        pq.pop();
        pq.emplace(node->index, squared_dist);
    }

    // 确定搜索顺序
    float dist_to_plane = distanceToSplitPlane(node, query);
    bool go_left = query[node->axis] < node->point[node->axis];

    // 递归搜索最可能的子树
    searchKNN(go_left ? node->left.get() : node->right.get(),
             query, k, pq);

    // 检查是否需要搜索另一个子树
    float worst_dist = pq.size() < static_cast<size_t>(k) ?
                      std::numeric_limits<float>::max() :
                      pq.top().distance;
                      
    if (dist_to_plane * dist_to_plane <= worst_dist) {
        searchKNN(go_left ? node->right.get() : node->left.get(),
                 query, k, pq);
    }
}
}//namespace bpa

