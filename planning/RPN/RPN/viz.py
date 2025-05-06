import trimesh
import os

# Define the directory containing the mesh files
mesh_dir = "rpn/experiment_dir/registered_meshes/"

# Get all the .obj files in the directory
obj_files = [f for f in os.listdir(mesh_dir) if f.endswith(".obj")]

# Iterate through each .obj file
for obj_file in obj_files:
    # Construct the base name (e.g., 0, 2, 4)
    base_name = obj_file.split(".")[0]
    
    # Construct paths for the corresponding .mtl and .png files
    obj_file_path = os.path.join(mesh_dir, obj_file)
    mtl_file = os.path.join(mesh_dir, f"{base_name}_material.mtl")
    texture_file = os.path.join(mesh_dir, f"{base_name}_texture.png")
    
    # Load the .obj file (trimesh will automatically handle the associated .mtl file)
    mesh = trimesh.load_mesh(obj_file_path, force='mesh')
    
    # Check if the mesh is loaded correctly
    if not mesh.is_empty:
        # If the mesh has an associated texture (check for the presence of a texture in the material)
        if mesh.visual.kind == 'texture' and texture_file:
            print(f"Visualizing {obj_file} with texture: {texture_file}")
        
        # Show the mesh using Trimesh's built-in viewer
        mesh.show()
    else:
        print(f"Failed to load {obj_file}")
