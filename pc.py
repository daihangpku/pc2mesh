import numpy as np
import os
import open3d as o3d

def generate_cylinder_ply(filename, radius=1.0, height=2.0, point_distance=0.05, noise_std=0.02, visualize=True):
    # Calculate number of points needed along each dimension
    circumference = 2 * np.pi * radius
    n_points_circle = int(circumference / point_distance)
    n_points_height = int(height / point_distance)
    
    # Generate points
    theta = np.linspace(0, 2*np.pi, n_points_circle, endpoint=False)
    h = np.linspace(-height/2, height/2, n_points_height)
    
    # Create meshgrid
    THETA, H = np.meshgrid(theta, h)
    
    # Calculate points
    x = radius * np.cos(THETA)
    y = radius * np.sin(THETA)
    z = H
    
    # Reshape to (n_points x 3) array
    points = np.zeros((n_points_circle * n_points_height, 3))
    points[:, 0] = x.flatten()
    points[:, 1] = y.flatten()
    points[:, 2] = z.flatten()
    
    # Add random noise
    noise = np.random.normal(0, noise_std, points.shape)
    points += noise
    
    # Calculate normals (pointing outward from cylinder surface)
    normals = np.zeros_like(points)
    # For cylinder surface, normal is the normalized vector from axis to point
    normals[:, 0] = points[:, 0] - 0  # x component (axis is at x=0)
    normals[:, 1] = points[:, 1] - 0  # y component (axis is at y=0)
    normals[:, 2] = 0  # z component (parallel to axis)
    
    # Normalize normals
    norms = np.linalg.norm(normals, axis=1)
    normals = normals / norms[:, np.newaxis]
    
    # Create colors based on height and angle
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    # Map height to red channel
    colors[:, 0] = ((points[:, 2] + height/2) / height * 255).astype(np.uint8)
    # Map angle to green channel
    angles = np.arctan2(points[:, 1], points[:, 0])
    colors[:, 1] = ((angles + np.pi) / (2 * np.pi) * 255).astype(np.uint8)
    # Map radius variation to blue channel
    radii = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    colors[:, 2] = ((radii - radius + noise_std) / (2 * noise_std) * 255).astype(np.uint8)
    
    # Print statistics
    print(f"Point spacing parameters:")
    print(f"Target point distance: {point_distance}")
    print(f"Points around circumference: {n_points_circle}")
    print(f"Points along height: {n_points_height}")
    print(f"Total points: {len(points)}")
    
    # Calculate actual average distance between neighboring points
    def get_nearest_neighbor_distance(points, k=4):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        
        distances = []
        for i in range(min(100, len(points))):  # Sample 100 points for speed
            _, idx, dist = kdtree.search_knn_vector_3d(points[i], k+1)  # k+1 because point itself is included
            distances.extend(np.sqrt(dist[1:]))  # Exclude distance to self
            
        return np.mean(distances)
    
    avg_distance = get_nearest_neighbor_distance(points)
    print(f"Average distance to nearest neighbors: {avg_distance:.3f}")

    # Write PLY file
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property float nx\n")
        f.write("property float ny\n")
        f.write("property float nz\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        
        for i in range(len(points)):
            f.write(f"{points[i,0]:.6f} {points[i,1]:.6f} {points[i,2]:.6f} ")
            f.write(f"{normals[i,0]:.6f} {normals[i,1]:.6f} {normals[i,2]:.6f} ")
            f.write(f"{colors[i,0]} {colors[i,1]} {colors[i,2]}\n")

    if visualize:
        # Create point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.normals = o3d.utility.Vector3dVector(normals)
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(float) / 255.0)

        # Create coordinate frame
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0])

        # Visualize point cloud
        print("\nVisualization:")
        print("Red: height position")
        print("Green: angle position")
        print("Blue: radius variation")
        o3d.visualization.draw_geometries([pcd, coordinate_frame],
                                        window_name="Cylinder Point Cloud",
                                        width=800,
                                        height=600)

# Generate the point cloud
current_dir = os.getcwd()
output_path = os.path.join(current_dir, "cylinder.ply")
print(f"Generating PLY file at: {output_path}")
generate_cylinder_ply(
    output_path,
    radius=1.0,
    height=2.0,
    point_distance=0.05,  # 控制点之间的目标距离
    noise_std=0.02,      # 控制噪声大小
    visualize=True
)