#!/us/bin/env python3
import numpy as np
import pigpio
import sys
# import geopy


from geopy import distance
sys.path.append('/home/user/Nozzle_Control/nozzle_env/lib/python3.11/site-packages')


from pymavlink import mavutil


import time


# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
exit(0)


# Define GPIO pins for the servos
SERVO_PIN1 = 12
SERVO_PIN2 = 13


current_angle1 = 45
current_angle2 = 45


def set_servo1_angle(servo_pin, angle):
# Convert angle to PWM value
# print("angle1",angle)


duty_cycle = int((angle / 180.0) * 2000 + 500)
pi.set_servo_pulsewidth(servo_pin, duty_cycle)


def set_servo2_angle(servo_pin, angle):
# Convert angle to PWM value
# print("angle2",angle)
duty_cycle = int((angle / 180.0) * 2000 + 500)
pi.set_servo_pulsewidth(servo_pin, duty_cycle)


def smooth_move_servo(set_servo_angle, servo_pin, current_angle, target_angle):
# while abs(target_angle - current_angle) > SMOOTHING_STEP:
# # Move in small steps towards the target angle
# step = SMOOTHING_STEP if target_angle > current_angle else -SMOOTHING_STEP
# current_angle += step


# set_servo_angle(servo_pin, current_angle)
# time.sleep(UPDATE_INTERVAL)


# Set final angle
set_servo_angle(servo_pin, target_angle)


def move_servos(angle1, angle2):
global current_angle1, current_angle2


smooth_move_servo(set_servo1_angle, SERVO_PIN1, current_angle1, max(10,min(80,angle1 +45)))
smooth_move_servo(set_servo2_angle, SERVO_PIN2, current_angle2, max(0,min(180,angle2+90)))


# smooth_move_servo(set_servo2_angle, SERVO_PIN2, current_angle2, angle2 )


# smooth_move_servo(set_servo1_angle, SERVO_PIN1, current_angle1, 45)
# smooth_move_servo(set_servo2_angle, SERVO_PIN2, current_angle2, 90)


# set_servo1_angle(angle1 - 90, SERVO_PIN1)
# set_servo2_angle(angle2 + 45,SERVO_PIN2)


current_angle1 = angle1 + 45
current_angle2 = angle2 - 90



# def haversine_distance(coord1, coord2):
# distance_thing = distance.distance(coord1,coord2)
# print(distance_thing)
# return distance_thing * 1000 #km to m


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
x = distance * np.sin(bearing_rad) # East-West component
y = distance * np.cos(bearing_rad) # North-South component



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




def continuously_receive_data(connection_string):
# Connect to the Vehicle
master = mavutil.mavlink_connection(connection_string)



# Wait for the first heartbeat
# This ensures the connection is established
master.wait_heartbeat()


# Function to request messages at a specific interval
def request_message_interval(message_id: int, frequency_hz: float):
master.mav.command_long_send(
master.target_system, master.target_component,
mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
message_id,
1e6 / frequency_hz,
0, 0, 0, 0,
0
)


# Request specific messages at desired frequencies
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, 50) # Example frequency 50Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 50) # Example frequency 50Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50) # Example frequency 50Hz



gps_data = None
attitude_data = None
position_targets = [] # List to store the last two position targets


temp_end_target = {
'lat':0,
'lon': 0,
'alt': 0
}



try:
while True: # Infinite loop
# Fetch a new message
msg = master.recv_match(blocking=True)



if msg is not None:
# Process message based on its type
if msg.get_type() == 'GPS_RAW_INT':
gps_data = {
'lat': msg.lat / 1e7,
'lon': msg.lon / 1e7,
'alt': msg.alt / 1e3
}
# print(gps_data)
elif msg.get_type() == 'ATTITUDE':
attitude_data = {
'roll': msg.roll,
'pitch': msg.pitch,
'yaw': msg.yaw
}
# print(attitude_data)
elif msg.get_type() == 'POSITION_TARGET_GLOBAL_INT':
# elif msg.get_type() == 'GPS_RAW_INT':
# gps_data = {
# 'lat': msg.lat / 1e7,
# 'lon': msg.lon / 1e7,
# 'alt': msg.alt / 1e3
# }
# print(gps_data)
# Process the current GPS data
target_data = {
'lat': msg.lat_int / 1e7, # Convert latitude to degrees
'lon': msg.lon_int / 1e7, # Convert longitude to degrees
'alt': msg.alt / 1e3 # Convert altitude to kilometers
}
# target_data ={'lat': 2.8582875, 'lon': 101.3920875, 'alt': 12.64}


# print("target",target_data) # Debug: print the current GPS data


# Check if the current data is different from the last stored data
if temp_end_target != target_data:
# Append the new data
position_targets.append(target_data)

# If there are more than two data points, remove the oldest one
if len(position_targets) > 2:
position_targets.pop(0)


# Update the temporary variable with the latest data
temp_end_target = target_data


if position_targets:
target1 = position_targets[0]
target2 = position_targets[-1]
distance1, bearing1 = distance_and_bearing((gps_data['lat'], gps_data['lon']), (target1['lat'], target1['lon']))
distance2, bearing2 = distance_and_bearing((gps_data['lat'], gps_data['lon']), (target2['lat'], target2['lon']))



if distance1 <30 or distance2 <30:


if distance1 <= distance2:
x, y = distance_bearing_to_xy(distance1, bearing1)
else:
x, y = distance_bearing_to_xy(distance2, bearing2)


rotated_vector = rotate_vector([x, y, -1], attitude_data['roll'], attitude_data['pitch'], attitude_data['yaw'])
# unrotated = [x, y, -1]
anglexz, angleyz = angles_with_planes(rotated_vector)


# Check if the angles are above 1 degree
if abs(anglexz) > 1 or abs(angleyz) > 1:
# print("Moving servos. Anglexz: %f, Angleyz: %f" % (anglexz, angleyz))
move_servos(anglexz, angleyz)
else:
print("Angles are too small. Ignoring servo movement.")
# move_servos(0,0)


else:
print('distance too small')

# move_servos(0,0)


else:
# swing_servos(0,90)
print("No data received")



# Print data (or process it as needed)
# print("GPS Data:", gps_data)
# print("Attitude Data:", attitude_data)
# print("Position Targets:", position_targets)



# Sleep for a short duration to prevent high CPU usage
# time.sleep(0.1)



except KeyboardInterrupt:
print("Stopping data reception...")



# Example usage
# connection_string = '/dev/ttyACM0'
# continuously_receive_data(connection_string)



#
continuously_receive_data("udp:192.168.0.163:14550")




# cProfile.run(continuously_receive_data("udp:192.168.0.123:14550"),"profiling_data.prof")


# move_servos(0,0)