#include <Arduino.h>
#include <Servo.h>
#include <mavlink.h>

const float EarthRadiusKm = 6371.0;

// lat lons
float currentDronePosition_latitude = 0 ;
float currentDronePosition_longitude = 0 ;

float previousTargetPosition_latitude = 0;
float previousTargetPosition_longitude = 0;

float currentTargetPosition_latitude = 0;
float currentTargetPosition_longitude = 0;


float currentAttitude_roll = 0;
float currentAttitude_pitch = 0;
float currentAttitude_yaw = 0;

// Servos
Servo servo1;
Servo servo2;

// Constants
const unsigned long MAVLINK_INTERVAL = 1000;
const uint8_t NUM_HEARTBEATS_WAIT = 60; 
const uint8_t MAX_WAYPOINTS = 2;      

// MAVLink Streams
const uint8_t MAVStreams[] PROGMEM = {MAV_DATA_STREAM_ALL};
const uint8_t MAVRates[] PROGMEM = {0x02, 0x05, 0x02}; 
const uint8_t NUM_STREAMS = sizeof(MAVStreams) / sizeof(MAVStreams[0]);

unsigned long previousMillisMAVLink = 0;
unsigned long printing_millis = 0;
int numHeartbeatsPassed = 0;

// Function prototypes
void requestMAVLinkData();
void processIncomingMAVLink();
float HaversineDistance(float, float, float, float);
float Bearing(float, float, float, float);


void ConstructVectorFromDistanceAndBearing(float distance, float bearing, float &x_direction,float  &y_direction) {
    double bearingRad = degrees(bearing);
    x_direction = distance * sin(bearingRad); // East-West component
    y_direction = distance * cos(bearingRad); // North-South component
}

void CalculatePlaneAngles(float x, float y, float z, float &xz_angle, float &yz_angle) {
    xz_angle = atan2(z, x); // Angle in the xz-plane
    yz_angle = atan2(z, y); // Angle in the yz-plane

    // Convert radians to degrees
    xz_angle = xz_angle * 180.0 / M_PI;
    yz_angle = yz_angle * 180.0 / M_PI;
}

void setup() {
    Serial.begin(57600);

    servo1.attach(9);
    servo2.attach(10);
}

void loop() {
    unsigned long currentMillis = millis();

    // Send heartbeat at regular intervals
    if (currentMillis - previousMillisMAVLink >= MAVLINK_INTERVAL) {
        previousMillisMAVLink = currentMillis;

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_heartbeat_pack(1, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial.write(buf, len);

        numHeartbeatsPassed++;
        if (numHeartbeatsPassed >= NUM_HEARTBEATS_WAIT) {
            requestMAVLinkData();
            numHeartbeatsPassed = 0;
        }
    }

    // Process incoming MAVLink messages
    processIncomingMAVLink();

        printing_millis = currentMillis;

        float x_direction = 0;
        float y_direction = 0;

        float distance_forward = HaversineDistance(currentDronePosition_latitude, currentDronePosition_longitude, currentTargetPosition_latitude, currentTargetPosition_longitude);
        float distance_backward = HaversineDistance(currentDronePosition_latitude, currentDronePosition_longitude, previousTargetPosition_latitude, previousTargetPosition_longitude);
        
        if (distance_forward < distance_backward) {
          float bearing_forward = Bearing(currentDronePosition_latitude, currentDronePosition_longitude, currentTargetPosition_latitude, currentTargetPosition_longitude);

          ConstructVectorFromDistanceAndBearing(distance_forward, bearing_forward, x_direction, y_direction);
        }
        else if (distance_backward < distance_forward) {
          float bearing_backward = Bearing(currentDronePosition_latitude, currentDronePosition_longitude, previousTargetPosition_latitude, previousTargetPosition_longitude);

          ConstructVectorFromDistanceAndBearing(distance_backward, bearing_backward, x_direction, y_direction);
        }

        float z = 2; //CHANGE THIS TO BE THE RANGEFINDER DISTANCE
        
        float xz_angle = 0;
        float yz_angle = 0;


        CalculatePlaneAngles(x_direction, y_direction, z, xz_angle, yz_angle);

        servo1.write(xz_angle);
        servo2.write(yz_angle);

        Serial.print("XZ Angle: ");
        Serial.println(xz_angle);
        Serial.print("YZ Angle: ");
        Serial.println(yz_angle);
    
}



void requestMAVLinkData() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    for (int i = 0; i < NUM_STREAMS; i++) {
        mavlink_msg_request_data_stream_pack(1, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial.write(buf, len);
    }
}


void processIncomingMAVLink() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
      break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t global_position;
            mavlink_msg_global_position_int_decode(&msg, &global_position);
            // Add the following lines
            currentDronePosition_latitude = global_position.lat / 1e7 ; // Convert to degrees
            currentDronePosition_longitude = global_position.lon / 1e7 ; // Convert to degrees

            break;
        }

        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
            mavlink_position_target_global_int_t target_global_int;
            mavlink_msg_position_target_global_int_decode(&msg, &target_global_int);

            float newLatitude = target_global_int.lat_int / 1e7; 
            float newLongitude = target_global_int.lon_int / 1e7; 

            // Check if the new position is the same as the current position
            bool isSamePosition = (abs(newLatitude - currentTargetPosition_latitude) == 0) &&
                                (abs(newLongitude - currentTargetPosition_longitude) == 0);

            if (!isSamePosition) {
                // Store the current target position as the previous one before updating
                previousTargetPosition_latitude = currentTargetPosition_latitude;
                previousTargetPosition_longitude = currentTargetPosition_longitude;

                // Update the current target position
                currentTargetPosition_latitude = newLatitude;
                currentTargetPosition_longitude = newLongitude;
            }

            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: { // #30
          mavlink_attitude_t attitude;
          mavlink_msg_attitude_decode(&msg, &attitude);
          currentAttitude_roll = attitude.roll;
          currentAttitude_pitch = attitude.pitch;
          currentAttitude_yaw = attitude.yaw;
          break;
        }
        
       default:
          break;
      }
    }
  }
}



float HaversineDistance(float lat1, float lon1, float lat2, float lon2) {
    lat1 = degrees(lat1);
    lon1 = degrees(lon1);
    lat2 = degrees(lat2);
    lon2 = degrees(lon2);

    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;

    float a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1) * cos(lat2) * 
               sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EarthRadiusKm * c;
}

float Bearing(float lat1, float lon1, float lat2, float lon2) {
    lat1 = degrees(lat1);
    lon1 = degrees(lon1);
    lat2 = degrees(lat2);
    lon2 = degrees(lon2);

    float dLon = lon2 - lon1;

    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - 
               sin(lat1) * cos(lat2) * cos(dLon);
    float brng = atan2(y, x);

    return fmod((degrees(brng) + 360), 360);
}
