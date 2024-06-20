import bpy
import math
import numpy as np

# Function to animate the object in a circle
def animate_RSO_and_chaser_circular(RSO_name, chaser_name, orbit_radius, inspection_radius, frames):
    # Ensure we're dealing with the object we expect
    RSO = bpy.data.objects[RSO_name]
    chaser = bpy.data.objects[chaser_name]

    bpy.context.view_layer.objects.active = RSO
    RSO.select_set(True)
    
    LVLH_matrices = []
    
    # Number of frames per full circle (one loop)
    total_frames = frames
    for frame in range(total_frames + 1):
        angle = 2 * np.pi() * frame / total_frames
        x = orbit_radius * np.cos(angle)
        y = orbit_radius * np.sin(angle)
        z = 0  # Z-coordinate remains constant
        
        # Set the object's location
        RSO.location = (x, y, z)
        # Insert a keyframe
        RSO.keyframe_insert(data_path="location", frame=frame)
        
        Earth_center = np.array(bpy.data.objects['Earth'].location)
        RSO_center = np.array(bpy.data.objects['RSO'].location)
        
        z_lvlh_vector = Earth_center - RSO_center
        RSO_z_lvlh = np.linalg.norm(z_lvlh_vector)
        RSO_x_lvlh = 0
        RSO_y_lvlh = 0
        
        z_lvlh_unit = z_lvlh_vector/np.linalg.norm(z_lvlh_vector)
        x_lvlh_unit = np.array([-math.cos(angle), math.sin(angle), 0])
        y_lvlh_unit = np.cross(z_lvlh_unit, x_lvlh_unit)
        
        xyz_to_lvlh_matrix = np.vstack([x_lvlh_unit.T, y_lvlh_unit.T, z_lvlh_unit.T])
        print(xyz_to_lvlh_matrix)
        
        LVLH_matrices.append(xyz_to_lvlh_matrix)
        
    for frame in range(total_frames + 1):
        angle = 2 * math.pi * frame / total_frames
        x = inspection_radius * math.cos(4*angle) + RSO.location.x
        y = inspection_radius * math.sin(4*angle) + RSO.location.y
        z = 0
        
        # Set camera's location
        chaser.location = (x, y, z)
        chaser.keyframe_insert(data_path="location", frame=frame)

animate_RSO_and_chaser_circular('RSO', 'Chaser', 113.78, 2, 360)