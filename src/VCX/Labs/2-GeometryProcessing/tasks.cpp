#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/ball_pivot.h"
#include "Labs/2-GeometryProcessing/mesh.h"
#include "Labs/2-GeometryProcessing/kdtree.h"
#include "Labs/2-GeometryProcessing/io.h"
namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 5. Marching Cubes *****************/
    // 实现pc2mesh函数
    void pc2mesh(Engine::SurfaceMesh& output, 
                    const std::string& data_path, 
                    const double r) {
        // 1. 读取点云数据
        bpa::PointCloud cloud;
        bpa::io::PointCloudIO::readPointCloud(data_path, cloud);
        
        cloud.buildKDTree();
        // 2. 预处理点云
        //cloud.computeNormals(10); // 计算法向量
        // cloud.removeOutliers(10, 2.0f); // 移除离群点
        
        // 3. 确定合适的球半径
        float radius = r; // 根据网格大小调整球半径
        
        // 4. 执行Ball Pivoting算法
        bpa::BallPivoting bpa(cloud, radius);
        if (!bpa.reconstruct()) {
            throw std::runtime_error("Failed to reconstruct mesh");
        }
        
        // 5. 将结果转换为输出网格
        bpa::convertToMesh(bpa, output);
    }
} // namespace VCX::Labs::GeometryProcessing
