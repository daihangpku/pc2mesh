// io.h
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include "point_cloud.h"

namespace VCX::Labs{
namespace io {

enum class FileFormat {
    PLY,
    OBJ,
    XYZ,
    UNKNOWN
};

class PointCloudIO {
public:
    // 读取点云文件
    static bool readPointCloud(const std::string& filename, PointCloud& cloud);
    
    // 保存点云文件
    static bool writePointCloud(const std::string& filename, const PointCloud& cloud);

private:
    // 获取文件格式
    static FileFormat getFileFormat(const std::string& filename);
    
    // 读取不同格式的文件
    static bool readPLY(const std::string& filename, PointCloud& cloud);
    static bool readOBJ(const std::string& filename, PointCloud& cloud);
    static bool readXYZ(const std::string& filename, PointCloud& cloud);
    
    // 写入不同格式的文件
    static bool writePLY(const std::string& filename, const PointCloud& cloud);
    static bool writeOBJ(const std::string& filename, const PointCloud& cloud);
    static bool writeXYZ(const std::string& filename, const PointCloud& cloud);
};

// 实现文件
inline FileFormat PointCloudIO::getFileFormat(const std::string& filename) {
    std::string ext = filename.substr(filename.find_last_of(".") + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if (ext == "ply") return FileFormat::PLY;
    if (ext == "obj") return FileFormat::OBJ;
    if (ext == "xyz") return FileFormat::XYZ;
    return FileFormat::UNKNOWN;
}

inline bool PointCloudIO::readPointCloud(const std::string& filename, PointCloud& cloud) {
    FileFormat format = getFileFormat(filename);
    
    switch (format) {
        case FileFormat::PLY:
            return readPLY(filename, cloud);
        case FileFormat::OBJ:
            return readOBJ(filename, cloud);
        case FileFormat::XYZ:
            return readXYZ(filename, cloud);
        default:
            std::cerr << "Unsupported file format: " << filename << std::endl;
            return false;
    }
}

inline bool PointCloudIO::readPLY(const std::string& filename, PointCloud& cloud) {
    //std::ifstream file(filename, std::ios::binary);
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    // PLY header parsing
    std::string line;
    int vertex_count = 0;
    bool has_normal = false;
    
    // Read header
    while (std::getline(file, line)) {
        if (line.find("element vertex") != std::string::npos) {
            sscanf(line.c_str(), "element vertex %d", &vertex_count);
        }
        else if (line.find("property float nx") != std::string::npos) {
            has_normal = true;
        }
        else if (line == "end_header") {
            break;
        }
    }

    // Read vertex data
    cloud.points.resize(vertex_count);
    if (has_normal) {
        cloud.normals.resize(vertex_count);
    }

    for (int i = 0; i < vertex_count; ++i) {
        float x, y, z;
        file >> x >> y >> z;
        cloud.points[i] = Eigen::Vector3f(x, y, z);
        
        if (has_normal) {
            float nx, ny, nz;
            file >> nx >> ny >> nz;
            cloud.normals[i] = Eigen::Vector3f(nx, ny, nz);
        }
    }

    return true;
}

inline bool PointCloudIO::readXYZ(const std::string& filename, PointCloud& cloud) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    cloud.points.clear();
    cloud.normals.clear();

    float x, y, z;
    while (file >> x >> y >> z) {
        cloud.points.push_back(Eigen::Vector3f(x, y, z));
        
        // Check if file contains normals
        if (file.peek() != '\n') {
            float nx, ny, nz;
            if (file >> nx >> ny >> nz) {
                cloud.normals.push_back(Eigen::Vector3f(nx, ny, nz));
            }
        }
    }

    return true;
}

inline bool PointCloudIO::writePointCloud(const std::string& filename, const PointCloud& cloud) {
    FileFormat format = getFileFormat(filename);
    
    switch (format) {
        case FileFormat::PLY:
            return writePLY(filename, cloud);
        case FileFormat::OBJ:
            return writeOBJ(filename, cloud);
        case FileFormat::XYZ:
            return writeXYZ(filename, cloud);
        default:
            std::cerr << "Unsupported file format: " << filename << std::endl;
            return false;
    }
}

inline bool PointCloudIO::writePLY(const std::string& filename, const PointCloud& cloud) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    // Write PLY header
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << cloud.points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    
    if (!cloud.normals.empty()) {
        file << "property float nx\n";
        file << "property float ny\n";
        file << "property float nz\n";
    }
    
    file << "end_header\n";

    // Write vertex data
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        file << cloud.points[i].x() << " "
             << cloud.points[i].y() << " "
             << cloud.points[i].z();
        
        if (!cloud.normals.empty()) {
            file << " " << cloud.normals[i].x() << " "
                 << cloud.normals[i].y() << " "
                 << cloud.normals[i].z();
        }
        
        file << "\n";
    }

    return true;
}
inline bool PointCloudIO::readOBJ(const std::string& filename, PointCloud& cloud) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    cloud.points.clear();
    cloud.normals.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "v") {  // Vertex
            float x, y, z;
            if (iss >> x >> y >> z) {
                cloud.points.push_back(Eigen::Vector3f(x, y, z));
            }
        }
        else if (token == "vn") {  // Normal
            float nx, ny, nz;
            if (iss >> nx >> ny >> nz) {
                cloud.normals.push_back(Eigen::Vector3f(nx, ny, nz).normalized());
            }
        }
        // Skip other OBJ elements (faces, texture coordinates, etc.)
    }

    // Verify data consistency
    if (!cloud.normals.empty() && cloud.normals.size() != cloud.points.size()) {
        std::cerr << "Warning: Number of normals doesn't match number of vertices" << std::endl;
        cloud.normals.clear();  // Clear normals if inconsistent
    }

    return !cloud.points.empty();
}

inline bool PointCloudIO::writeOBJ(const std::string& filename, const PointCloud& cloud) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    // Write header comment
    file << "# OBJ file created by BPA\n";
    file << "# Vertices: " << cloud.points.size() << "\n";

    // Write vertices
    for (const auto& point : cloud.points) {
        file << "v " << point.x() << " " 
                    << point.y() << " "
                    << point.z() << "\n";
    }

    // Write normals if they exist
    if (!cloud.normals.empty()) {
        file << "# Normals: " << cloud.normals.size() << "\n";
        for (const auto& normal : cloud.normals) {
            file << "vn " << normal.x() << " "
                        << normal.y() << " "
                        << normal.z() << "\n";
        }

        // Write vertex-normal associations
        // In OBJ format, this would typically be done through faces
        // Since we're just storing point cloud data, we'll skip face definitions
    }

    return true;
}

inline bool PointCloudIO::writeXYZ(const std::string& filename, const PointCloud& cloud) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    file.precision(8);  // Set precision for floating point output
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        // Write vertex coordinates
        file << cloud.points[i].x() << " "
             << cloud.points[i].y() << " "
             << cloud.points[i].z();

        // Write normal if available
        if (!cloud.normals.empty()) {
            file << " " << cloud.normals[i].x() << " "
                 << cloud.normals[i].y() << " "
                 << cloud.normals[i].z();
        }

        file << "\n";
    }

    return true;
}
} // namespace io
} // namespace bpa