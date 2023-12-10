from pymavlink import mavutil
import calculations as calc
import servo_control

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
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, 10) # Example frequency 10Hz
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 10) # Example frequency 10Hz
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10) # Example frequency 10Hz


    gps_data = None
    attitude_data = None
    position_targets = []

    temp_end_target = {
                        'lat':0,
                        'lon': 0,
                        'alt': 0
                    }

    try:
        while True:  # Infinite loop
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
                    print(gps_data)
                elif msg.get_type() == 'ATTITUDE':
                    attitude_data = {
                        'roll': msg.roll,
                        'pitch': msg.pitch,
                        'yaw': msg.yaw
                    }
                    print(attitude_data)
                elif msg.get_type() == 'POSITION_TARGET_GLOBAL_INT':

                    target_data = {
                        'lat': msg.lat_int / 1e7,  # Convert latitude to degrees
                        'lon': msg.lon_int / 1e7,  # Convert longitude to degrees
                        'alt': msg.alt / 1e3      # Convert altitude to kilometers
                    }

                    # test data
                    # target_data ={'lat': 2.8582875, 'lon': 101.3920875, 'alt': 12.64}

                    print("target",target_data)  # Debug: print the current GPS data

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
                        distance1, bearing1 = calc.distance_and_bearing((gps_data['lat'], gps_data['lon']), (target1['lat'], target1['lon']))
                        distance2, bearing2 = calc.distance_and_bearing((gps_data['lat'], gps_data['lon']), (target2['lat'], target2['lon']))


                        if distance1 <30 or distance2 <30:

                            if distance1 <= distance2:
                                x, y = calc.distance_bearing_to_xy(distance1, bearing1)
                            else:
                                x, y = calc.distance_bearing_to_xy(distance2, bearing2)

                            rotated_vector = calc.rotate_vector([x, y, -1], attitude_data['roll'], attitude_data['pitch'], attitude_data['yaw'])
                            anglexz, angleyz = calc.angles_with_planes(rotated_vector)

                            # Check if the angles are above 1 degree
                            if abs(anglexz) > 1 or abs(angleyz) > 1:
                                print("Moving servos. Anglexz: %f, Angleyz: %f" % (anglexz, angleyz))
                                servo_control.move_servos(anglexz, angleyz)
                            else:
                                print("Angles are too small. Ignoring servo movement.")

                        else:
                            print('distance too small')
                            servo_control.move_servos(0,0)

            else:
                print("No data received")


    except KeyboardInterrupt:
        print("Stopping data reception...")


connection_string = '/dev/ttyACM0'
continuously_receive_data(connection_string)