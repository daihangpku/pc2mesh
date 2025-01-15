import numpy as np
import os

def generate_ply_file(filename, n_points=100):
    # Generate grid points in [-1, 1] range
    x = np.linspace(-1, 1, n_points)
    y = np.linspace(-1, 1, n_points)
    X, Y = np.meshgrid(x, y)
    
    # Create points
    points = np.zeros((n_points * n_points, 3))
    points[:, 0] = X.flatten()
    points[:, 1] = Y.flatten()
    points[:, 2] = np.random.normal(0, 0.00, n_points * n_points)  # Small z variation
    
    # Create normals (mostly pointing up with small variations)
    normals = np.zeros((n_points * n_points, 3))
    normals[:, 2] = 1.0  # Point up in Z direction
    # Add small random variations to X and Y components
    normals[:, 0] = np.random.normal(0, 0.0, n_points * n_points)
    normals[:, 1] = np.random.normal(0, 0.0, n_points * n_points)
    # Normalize
    norms = np.linalg.norm(normals, axis=1)
    normals = normals / norms[:, np.newaxis]
    
    # Create colors
    colors = np.zeros((n_points * n_points, 3), dtype=np.uint8)
    colors[:, 0] = 255  # Red
    colors[:, 1] = np.linspace(0, 255, n_points * n_points).astype(np.uint8)  # Varying green

    # Verify data
    print("Data verification:")
    print(f"Points shape: {points.shape}")
    print(f"Normals shape: {normals.shape}")
    print(f"Colors shape: {colors.shape}")
    print("\nFirst few points:")
    for i in range(3):
        print(f"Point {i}:")
        print(f"  Position: {points[i]}")
        print(f"  Normal: {normals[i]}")
        print(f"  Color: {colors[i]}")

    # Write PLY file
    with open(filename, 'w') as f:
        # Write header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n_points * n_points}\n")
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
        
        # Write data
        for i in range(n_points * n_points):
            # Ensure proper formatting with 6 decimal places
            f.write(f"{points[i,0]:.6f} {points[i,1]:.6f} {points[i,2]:.6f} ")
            f.write(f"{normals[i,0]:.6f} {normals[i,1]:.6f} {normals[i,2]:.6f} ")
            f.write(f"{colors[i,0]} {colors[i,1]} {colors[i,2]}\n")

# Generate the point cloud
current_dir = os.getcwd()
output_path = os.path.join(current_dir, "cube.ply")
print(f"Generating PLY file at: {output_path}")
generate_ply_file(output_path, n_points=20)