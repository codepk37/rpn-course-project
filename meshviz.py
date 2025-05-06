import open3d as o3d

# Load the saved .obj file
mesh = o3d.io.read_triangle_mesh("filtered.obj")

# Check if the mesh has vertex colors and display them if available
if mesh.has_vertex_colors():
    print("Mesh has vertex colors.")
else:
    print("Mesh does not have vertex colors.")

# Create a coordinate axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# Visualize the mesh with the axis
o3d.visualization.draw_geometries([mesh, axis], mesh_show_back_face=True, window_name="Filtered Mesh + Axis")
