#include "utils/io.h"
#include <iostream>
int main() {
    bpa::PointCloud cloud;
    
    // 读取点云文件
    if (bpa::io::PointCloudIO::readPointCloud("input.ply", cloud)) {
        std::cout << "Successfully loaded " << cloud.points.size() << " points." << std::endl;
        
        // 处理点云数据...
        
        // 保存点云文件
        if (bpa::io::PointCloudIO::writePointCloud("output.ply", cloud)) {
            std::cout << "Successfully saved point cloud." << std::endl;
        }
    }
    
    return 0;
}