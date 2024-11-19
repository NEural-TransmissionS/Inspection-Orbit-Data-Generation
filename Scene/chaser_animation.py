import bpy
import json
import math
import mathutils
import numpy as np
import os

MU = 398600441500000/1000**3

# animate RSO and chaser orbits
def animate_RSO_and_chaser(RSO_name, chaser_name,
                           orbit_radius, inspection_orbit, delta_x, z0=0,
                           total_frames=100., overhead_name=None,
                           output_path=False, export_extrinsics=False):
                                       
    # select Blender objects
    RSO = bpy.data.objects[RSO_name]
    chaser = bpy.data.objects[chaser_name]
    if overhead_name:
        overhead_cam = bpy.data.objects[overhead_name]

    # angular velocity
    omega = np.sqrt(MU / orbit_radius**3)

    # times
    if inspection_orbit == 'VBAR':
        times = np.linspace(0, 4*np.pi/omega, total_frames)
    else:
        times = np.linspace(0, 2*np.pi/omega, total_frames)

    # generate chaser orbit in lvlh
    x_co, y_co, z_co = generate_orbits(inspection_orbit, times, omega, delta_x, z0)        
   
    if output_path:
        if not os.path.exists(output_path):
            os.mkdir(output_path)
           
        # save camera intrinsics
        camera_intrinsics_to_file(chaser, output_path + f'intrinsics.txt')
        camera_parameters = []
       
        # render settings
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.frame_start = 1
        bpy.context.scene.frame_end = len(times)
   
    # simulate one RSO orbit
    for frame, t in enumerate(times):
        xyz = np.array([orbit_radius * np.cos(omega*t), orbit_radius * np.sin(omega*t), 0])
       
        # Set the object's location and insert a keyframe
        RSO.location = xyz
        RSO.keyframe_insert(data_path="location", frame=frame)
       
        # LVLH to XYZ conversion
        lvlh_to_xyz = lvlh_to_xyz_matrix(RSO.location, orbit_radius, omega, t)
        lvlh = lvlh_to_xyz @ np.array([x_co[frame], y_co[frame], z_co[frame]]) + RSO.location
       
        # Set chaser's location and insert a keyframe
        chaser.location = lvlh
        chaser.keyframe_insert(data_path="location", frame=frame)
       
        # overhead cam
        if overhead_name:
            overhead_cam.location = lvlh + np.array([0, 1, 0.25])
            overhead_cam.keyframe_insert(data_path="location", frame=frame)
           
        if output_path:
            # Update the scene to ensure keyframes are processed
            bpy.context.view_layer.update()
               
            print(f'Frame {frame}\n')
           
            if export_extrinsics:
                # camera_extrinsics_to_file(chaser, output_path + f'extrinsics_frame_{frame:05}.txt')
                camera_parameters.append(create_extrinsics_json_entry(chaser, frame))
           
            bpy.context.scene.frame_set(frame)
            bpy.context.scene.render.filepath = output_path + f'frame_{frame:05}.png'
            bpy.ops.render.render(write_still=True)
           
    if output_path and export_extrinsics:
        with open(output_path + 'cameras.json', 'w') as json_file:
            json.dump(camera_parameters, json_file, indent=4)


def create_extrinsics_json_entry(camera, frame):
    position = camera.location
    rotation_matrix = camera.matrix_world.to_3x3()
   
    data = {
            'id': frame,
            'img_name': f'frame_{frame:05}',
            'width': bpy.context.scene.render.resolution_x,
            'height': bpy.context.scene.render.resolution_y,
            'position': [position.x, position.y, position.z],
            'rotation': [
                            [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2]],
                            [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2]],
                            [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]]
                        ],
            'fy': camera.data.lens,
            'fx': camera.data.lens
            }
           
    return data

def camera_extrinsics_to_file(camera, filepath):
    # Get the world matrix of the camera object
    world_matrix = camera.matrix_world

    # Extract the rotation and translation from the world matrix
    rotation_matrix = world_matrix.to_3x3()
    translation_vector = world_matrix.to_translation()

    # Convert rotation matrix to a 3x4 matrix
    extrinsic_matrix = mathutils.Matrix((
        (rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], translation_vector[0]),
        (rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], translation_vector[1]),
        (rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], translation_vector[2]),
        (0, 0, 0, 1)
    ))
   
    #print(f'Extrinsics: {extrinsic_matrix}')
       
def camera_intrinsics_to_file(camera, filepath):
    # Get the camera object
    camera = camera.data

    # Sensor width and height (in mm)
    sensor_width = camera.sensor_width
    sensor_height = camera.sensor_height

    # Focal length (in mm)
    focal_length = camera.lens

    # Image resolution (in pixels)
    scene = bpy.context.scene
    resolution_x = scene.render.resolution_x
    resolution_y = scene.render.resolution_y

    # Scale factors
    scale_x = scene.render.pixel_aspect_x
    scale_y = scene.render.pixel_aspect_y

    # Calculate the principal point (in pixels)
    principal_x = resolution_x / 2
    principal_y = resolution_y / 2

    # Intrinsic matrix
    intrinsic_matrix = mathutils.Matrix((
        (focal_length * resolution_x / sensor_width, 0, principal_x),
        (0, focal_length * resolution_y / sensor_height, principal_y),
        (0, 0, 1)
    ))
   
    # print(f'Intrinsics: {intrinsic_matrix}')

    # Save to file
    with open(filepath, "w") as file:
        file.write(str(intrinsic_matrix))
       
def generate_orbits(orbit_type, times, omega, delta_x, z0=0, orbit_csv=None):
    # Coplanar Circular Orbit
    if orbit_type == 'COPLANAR':
        # return CW(times, -delta_x, 0, 0, (3/2)*omega*z0, 0, 0, omega)
        return CW(times, -delta_x, 0, 0, 0, 0, 0, omega)
       
    # V-Bar Hop Orbit      
    elif orbit_type == 'VBAR':
        # initial change in velocity along x
        delta_v_x = -omega*delta_x/(6*np.pi)
        return CW(times, -delta_x/2, 0, 0, delta_v_x, 0, 0, omega)

    # NMC Orbit
    elif orbit_type == 'NMC':
        # initial change in velocity along z
        delta_v_z = omega*delta_x/4
        return CW(times, -delta_x/2, 0, 0, 0, 0, delta_v_z, omega)
    
    # Corkscrew Orbit
    elif orbit_type == 'CORKSCREW':
        # initial change in velocity along z
        inclination = 45 * np.pi / 180
        return CW(times, -delta_x, 0, 0, 0, delta_x*omega*np.sin(inclination), -delta_x*omega*np.sin(inclination), omega)
       
    # input orbit
    elif obit_type == 'CSV':
        return pd.read_csv(orbit_csv).to_numpy()


# solutions to the Clohessy-Wiltshire equations
def CW(t, x0, y0, z0, x0dot, y0dot, z0dot, omega):
    x = (4*x0dot/omega - 6*z0)*np.sin(omega*t) - 2*z0dot/omega*np.cos(omega*t) + (6*omega*z0 - 3*x0dot)*t + (x0 + 2*z0dot/omega)
    y = y0*np.cos(omega*t) + (y0dot/omega)*np.sin(omega*t)
    z = (2*x0dot/omega - 3*z0)*np.cos(omega*t) + z0dot/omega*np.sin(omega*t) + (4*z0 - 2*x0dot/omega)
    return x, y, z

def lvlh_to_xyz_matrix(RSO_center, orbit_radius, omega, t):
    # z_lvlh goes from RSO to Earth
    z_lvlh = bpy.data.objects['Earth'].location - RSO_center
    z_lvlh = z_lvlh/np.linalg.norm(z_lvlh)
   
    # x_lvlh is the orbit's tangent vector
    velocity = orbit_radius * omega * np.array([-np.sin(omega*t), np.cos(omega*t), 0])
    x_lvlh = velocity / np.linalg.norm(velocity)
   
    # y_lvlh is orthogonal to the others
    y_lvlh = np.cross(z_lvlh, x_lvlh)
   
    # return coordinate transformation matrix
    return np.array([x_lvlh, y_lvlh, z_lvlh]).T


simulations_folder = 'C:\Users\rwhite\Documents\GitHub'

#RSO = 'Cassini'
RSO = 'Sentinel6'
#orbit = ['NMC', 10]
orbit = ['CORKSCREW', 1]
#orbit = ['VBAR', 5]
#orbit = ['NMC', 50]
#orbit = ['COPLANAR', 5]
#dx = 7.5
#total_frames = 300
#altitude = ['LEO', 1138]
#altitude = ['LEO', 718]
altitude = ['LEO', 638 + 80.0]
#altitude = ['GEO', 638 + 3578.6]

#for RSO in ['Cassini']:
#    for orbit in ['NMC', 'VBAR', 'COPLANAR']:
    for orbit in ['CORKSCREW']:
        for ic in [1, 2.5, 5]:
        #for ic in [3, 7.5]:
            for altitude in [['LEO', 718], ['GEO', 36424]]:
               
                scenario_folder = f'{RSO}_{altitude[0]}_{orbit}_dxz0_{ic:04.2f}_dt/'
               
                output_path = simulations_folder + scenario_folder

                animate_RSO_and_chaser(RSO, 'Chaser Cam',
                                        altitude[1], orbit, delta_x=ic, z0=ic,
                                        total_frames=300, output_path=output_path, export_extrinsics=True)

#scenario_folder = f'{RSO}_{altitude[0]}_{altitude[1]}_{orbit}_dx_{dx:07.3f}_dt_{dt:07.3f}/'
#output_path = simulations_folder + scenario_folder            

#animate_RSO_and_chaser(RSO, 'Chaser Cam',
#                       altitude[1], orbit[0], delta_x=orbit[1], z0=orbit[1], total_frames=150,
#                       output_path=output_path, export_extrinsics=True)

#animate_RSO_and_chaser(RSO, 'Chaser Cam',
#                       altitude[1], orbit[0], delta_x=orbit[1], z0=orbit[1],
#                       total_frames=300)