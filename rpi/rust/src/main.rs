extern crate mavlink;
use std::env;
use std::sync::Arc;
use mavlink::error::MessageReadError;
use std::thread;
use std::time::Duration;
use std::f64::consts::PI;
use nalgebra::{Matrix3, Vector3};
use geoutils::{Location};

mod pwm_control;

#[derive(PartialEq, Clone, Copy, Debug)]
struct GpsCoordinate {
    latitude: f64,
    longitude: f64,
}

#[derive(Debug)]
struct Attitude {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

fn vincenty_distance(coord1: &GpsCoordinate, coord2: &GpsCoordinate) -> f64 {
    // Convert GpsCoordinate to geoutils::Location
    let location1 = Location::new(coord1.latitude, coord1.longitude);
    let location2 = Location::new(coord2.latitude, coord2.longitude);

    // Calculate the distance using Vincenty's formula (geoutils)
    match location1.distance_to(&location2) {
        Ok(distance) => distance.meters() *0.1, // Return distance in meters
        Err(_) => 0.0, // Handle error case, e.g., return 0.0 or some error value
    }
}

fn bearing(coord1: &GpsCoordinate, coord2: &GpsCoordinate) -> f64 {
    let lat1_rad = coord1.latitude.to_radians();
    let lat2_rad = coord2.latitude.to_radians();

    let dlon_rad = (coord2.longitude - coord1.longitude).to_radians();

    let y = dlon_rad.sin() * lat2_rad.cos();
    let x = lat1_rad.cos() * lat2_rad.sin() - lat1_rad.sin() * lat2_rad.cos() * dlon_rad.cos();

    y.atan2(x).to_degrees().rem_euclid(360.0)
}

fn calculate_vector_angles(vector: Vector3<f64>) -> (f64, f64) {
    let xz_angle = calculate_angle(vector.x, vector.z);
    let yz_angle = calculate_angle(vector.y, vector.z);

    let (xz_angle, yz_angle) = (xz_angle.rem_euclid(360.0), yz_angle.rem_euclid(360.0));
    (xz_angle, yz_angle)
}

fn calculate_angle(a: f64, b: f64) -> f64 {
    (a.atan2(b) * 180.0 / PI).abs() // Convert radians to degrees
}

fn rotate_vector(attitude_data : &Attitude, vector: Vector3<f64>) -> Vector3<f64> {
    let roll_matrix = Matrix3::new(
        1.0, 0.0, 0.0,
        0.0, attitude_data.roll.cos(), -attitude_data.roll.sin(),
        0.0, attitude_data.roll.sin(), attitude_data.roll.cos()
    );

    let pitch_matrix = Matrix3::new(
        attitude_data.pitch.cos(), 0.0, attitude_data.pitch.sin(),
        0.0, 1.0, 0.0,
        -attitude_data.pitch.sin(), 0.0, attitude_data.pitch.cos()
    );

    let yaw_matrix = Matrix3::new(
        attitude_data.yaw.cos(), -attitude_data.yaw.sin(), 0.0,
        attitude_data.yaw.sin(), attitude_data.yaw.cos(), 0.0,
        0.0, 0.0, 1.0
    );

    // Combine the rotations
    let rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix;

    // Apply rotation to the vector
    rotation_matrix * vector
}

fn main() {
    let args: Vec<_> = env::args().collect();

    if args.len() < 2 {
        println!(
            "Usage: mavlink-dump (tcpout|tcpin|udpout|udpin|udpbcast|serial|file):(ip|dev|path):(port|baud)"
        );
        return;
    }

    // It's possible to change the mavlink dialect to be used in the connect call
    let mut mavconn = mavlink::connect::<mavlink::ardupilotmega::MavMessage>(&args[1]).unwrap();

    // here as an example we force the protocol version to mavlink V1:
    // the default for this library is mavlink V2
    mavconn.set_protocol_version(mavlink::MavlinkVersion::V2);

    let vehicle = Arc::new(mavconn);
    vehicle
        .send(&mavlink::MavHeader::default(), &request_parameters())
        .unwrap();
    vehicle
        .send(&mavlink::MavHeader::default(), &request_stream())
        .unwrap();

    thread::spawn({
        let vehicle = vehicle.clone();
        move || loop {
            let res = vehicle.send_default(&heartbeat_message());
            if res.is_ok() {
                thread::sleep(Duration::from_secs(1));
            } else {
                println!("send failed: {res:?}");
            }
        }
    });

    // Initialize current_gps_coordinate with None
    let mut current_gps_coordinate: Option<GpsCoordinate> = None;
    let mut current_target_gps_coordinate: Option<GpsCoordinate> = None;
    let mut previous_target_gps_coordinate: Option<GpsCoordinate> = None;
    let mut current_attitude: Option<Attitude> = None;

    loop {
        match vehicle.recv() {
            Ok((_header, msg)) => {
                match msg {
                    mavlink::ardupilotmega::MavMessage::GPS_RAW_INT(gps_data) => {
                        // Update current_gps_coordinate
                        current_gps_coordinate = Some(GpsCoordinate {
                            latitude: gps_data.lat as f64 / 1_000_000.0,
                            longitude: gps_data.lon as f64 / 1_000_000.0,
                        });
                        // Uncomment below line for debugging
                        // println!("GPS Coordinates: {:?}", current_gps_coordinate);
                    }

                    mavlink::ardupilotmega::MavMessage::ATTITUDE(attitude_data) => {
                        // Update current_attitude
                        current_attitude = Some(Attitude {
                            roll: attitude_data.roll as f64,
                            pitch: attitude_data.pitch as f64,
                            yaw: attitude_data.yaw as f64,
                        });
                        // Uncomment below line for debugging
                        // println!("Attitude Data: {:?}", current_attitude);
                    }
                    mavlink::ardupilotmega::MavMessage::POSITION_TARGET_GLOBAL_INT(target_position) => {
                        if let Some(current_coord) = &current_gps_coordinate {
                            let target_latitude = target_position.lat_int as f64 / 1_000_000.0;
                            let target_longitude = target_position.lon_int as f64 / 1_000_000.0;
                            let target_gps_coordinate = GpsCoordinate {
                                latitude: target_latitude,
                                longitude: target_longitude,
                            };
                    
                            // Compare and update old target GPS coordinate
                            if Some(&target_gps_coordinate) != current_target_gps_coordinate.as_ref() {
                                // println!("New target GPS coordinate: {:?}", target_gps_coordinate);
                                previous_target_gps_coordinate = current_target_gps_coordinate.take();
                                current_target_gps_coordinate = Some(target_gps_coordinate);
                            } else {
                                // println!("Old target");
                            }
                    
                            // Calculate distances
                            // Inside the mavlink::ardupilotmega::MavMessage::POSITION_TARGET_GLOBAL_INT block

if let Some(current_target) = &current_target_gps_coordinate {
    let distance_to_current_target = vincenty_distance(&current_coord, current_target);
    let distance_to_previous_target = previous_target_gps_coordinate
        .as_ref()
        .map(|prev_target| vincenty_distance(&current_coord, prev_target))
        .unwrap_or(f64::MAX);

    // Determine the smaller distance
    let smaller_distance = distance_to_current_target.min(distance_to_previous_target);

    // Check if the bigger distance is less than 10
    if smaller_distance > 10.0 {
        println!("{}",smaller_distance);
        println!("Distance too big");
    } else {
        // Rest of the calculations as before
        let (distance, target_for_bearing) = if distance_to_current_target < distance_to_previous_target {
            println!("Closer to current target: {} meters", distance_to_current_target);
            (distance_to_current_target, current_target)
        } else {
            println!("Closer to previous target: {} meters", distance_to_previous_target);
            (distance_to_previous_target, previous_target_gps_coordinate.as_ref().unwrap())
        };

        let angle = bearing(&current_coord, target_for_bearing);
        let angle_rad = angle.to_radians();
                                let x_cartesian = distance * angle_rad.cos();
                                let y_cartesian = distance * angle_rad.sin();

                                let z_cartesian = 2.0; // Ensure this value is appropriate for your application
                                let vector = Vector3::new(x_cartesian, y_cartesian, z_cartesian);

                                if let Some(attitude) = &current_attitude {
                                    let rotated_vector = rotate_vector(attitude, vector);
                                    // Further processing with rotated_vector
                                    // println!("Rotated Vector: {:?}", rotated_vector);
                                    let (xz_angle, yz_angle) = calculate_vector_angles(rotated_vector);

                                    // use servo control

                                    match pwm_control::set_servo_angles(xz_angle, yz_angle){
                                        Result::Ok(()) => println!("Servo angles set successfully"),
                                        Result::Err(e) => println!("Error setting servo angles: {:?}", e),
                                    }
                                    // Uncomment below lines for debugging
                                    println!("Angles: ({}, {})", xz_angle, yz_angle);
                                    // println!("Cartesian Coordinates: ({}, {})", x_cartesian, y_cartesian);

                                } else {
                                    // Handle the case where current_attitude is None
                                    println!("Current Attitude not available.");
                                }
    }
}
                                
                            }

                     else {
                        println!("Current GPS coordinates not available.");
                    }
                }
                _ => {
                    // Handle other messages or ignore
                }
            }
        }
        Err(MessageReadError::Io(e)) => {
            if e.kind() == std::io::ErrorKind::WouldBlock {
                // No messages currently available to receive -- wait a while
                thread::sleep(Duration::from_secs(1));
                continue;
            } else {
                println!("recv error: {e:?}");
                break;
            }
        }
        // Messages that didn't get through due to parser errors are ignored
        _ => {}
    }
}
}

/// Create a heartbeat message using 'ardupilotmega' dialect
pub fn heartbeat_message() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

/// Create a message requesting the parameters list
pub fn request_parameters() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::PARAM_REQUEST_LIST(
        mavlink::ardupilotmega::PARAM_REQUEST_LIST_DATA {
            target_system: 0,
            target_component: 0,
        },
    )
}

/// Create a message enabling data streaming
pub fn request_stream() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::REQUEST_DATA_STREAM(
        mavlink::ardupilotmega::REQUEST_DATA_STREAM_DATA {
            target_system: 0,
            target_component: 0,
            req_stream_id: 0,
            req_message_rate: 10,
            start_stop: 1,
        },
    )
}