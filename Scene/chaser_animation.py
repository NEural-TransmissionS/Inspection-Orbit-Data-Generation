    import bpy
    import math
    import numpy as np


    MU = 398600441500000/10000**3

    # animate RSO and chaser orbits
    def animate_RSO_and_chaser(RSO_name, chaser_name, orbit_radius, inspection_orbit, delta_x, overhead_name=None, z0=0, dt=1.):
                                            
        # select Blender objects
        RSO = bpy.data.objects[RSO_name]
        chaser = bpy.data.objects[chaser_name]
        if overhead_name:
            overhead_cam = bpy.data.objects[overhead_name]

        # angular velocity
        omega = np.sqrt(MU / orbit_radius**3)

        # times
        times = np.arange(0, 4*np.pi/omega, dt)

        # generate chaser orbit in lvlh
        x_co, y_co, z_co = generate_orbits(inspection_orbit, times, omega, delta_x, z0)
        
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
            
    def generate_orbits(orbit_type, times, omega, delta_x, z0=0, orbit_csv=None):
        # Coplanar Circular Orbit
        if orbit_type == 'coplanar':
            return CW(times, -delta_x, 0, 0, (3/2)*omega*z0, 0, 0, omega)
            
        # V-Bar Hop Orbit       
        elif orbit_type == 'vbar':
            # initial change in velocity along x
            delta_v_x = -omega*delta_x/(6*np.pi)
            return CW(times, -delta_x/2, 0, 0, delta_v_x, 0, 0, omega)

        # NMC Orbit
        elif orbit_type == 'nmc':
            # initial change in velocity along z
            delta_v_z = omega*delta_x/4 
            return CW(times, -delta_x/2, 0, 0, 0, 0, delta_v_z, omega)
            
        # input orbit
        elif obit_type == 'csv':
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

    animate_RSO_and_chaser('Cassini', 'Chaser Cam',
                           1138, 'nmc', 7.5)