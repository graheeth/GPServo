import numpy as np

def haversine_distance(coord1, coord2):
    # Radius of the Earth in kilometers
    R = 6371.0
    
    lat1, lon1 = coord1
    lat2, lon2 = coord2


    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])


    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2.0)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    distance = R * c
    return distance * 1000 #km to m


def calculate_bearing(coord1, coord2):
    lat1, lon1 = coord1
    lat2, lon2 = coord2


    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])


    dlon = lon2 - lon1
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)


    initial_bearing = np.arctan2(x, y)
    initial_bearing = np.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360


    return compass_bearing


def distance_and_bearing(coord1, coord2):
    distance = haversine_distance(coord1, coord2)
    bearing = calculate_bearing(coord1, coord2)
    return distance, bearing


def distance_bearing_to_xy(distance, bearing):
    # Convert bearing to radians
    bearing_rad = np.radians(bearing)


    # Calculate x and y components
    x = distance * np.sin(bearing_rad)  # East-West component
    y = distance * np.cos(bearing_rad)  # North-South component


    return x, y


def angles_with_planes(vector):
    # Normal vectors of the xz and yz planes
    normal_xz = np.array([0, 1, 0])
    normal_yz = np.array([1, 0, 0])

    # vector_magn =np.linalg.norm(vector)
    # normalized_vector = vector / vector_magn

    # Calculating the angle between the vector and the normal vectors
    angle_with_xz_normal = np.arccos(np.dot(vector, normal_xz) / (np.linalg.norm(vector) * np.linalg.norm(normal_xz)))
    angle_with_yz_normal = np.arccos(np.dot(vector, normal_yz) / (np.linalg.norm(vector) * np.linalg.norm(normal_yz)))


    # Calculating the complementary angles (angles with the planes)
    angle_with_xz_plane = np.pi/2 - angle_with_xz_normal
    angle_with_yz_plane = np.pi/2 - angle_with_yz_normal


    # Converting to degrees
    angle_with_xz_plane_deg = np.degrees(angle_with_xz_plane)
    angle_with_yz_plane_deg = np.degrees(angle_with_yz_plane)


    return angle_with_xz_plane_deg, angle_with_yz_plane_deg


def rotate_vector(vector, roll, pitch, yaw):
    # Convert angles from degrees to radians
    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    # Rotation matrices for roll, pitch, and yaw
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))

    # Apply rotation to the vector
    rotated_vector = np.dot(R, vector)
    return rotated_vector