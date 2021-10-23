# App-Drone Communication Protocol
The protocol was reversed mainly by reverse-engineering the app, and also some network sniffing
of actual sent packets. Note that in the rest of this document structs are encoded in LE, and all
structs are packed. Sizes of fields are denoted by `uN` where N is the number of bits (or `iN` if the field is signed). If N is not divisble by 8, it is
part of a bitfield.

## Channels of Communication
The app communicates with the drone over UDP.
The drone acts as the router, so its IP is static: `192.168.10.1`.
The app maintains two sockets:
- Control Socket on drone port `8889`, along which commands, responses and drone updates are sent
- Video Socket which is by default on port `7777` (The app announces which port to use), along
which raw H264 packets are sent, wrapped by a minimal header for segmentation

## Control Socket
This connection is two-way: The app sends commands which the drone acknowledges and responds to,
but the drone can also query information, and the app responds accordingly. The connection is
initiated by a "Connection Request" and "Connection Acknowledge" pair, which take the form:
- `conn_req:PP` where PP are LE bytes of the video PORT to use
- `conn_ack:PP` where PP is the same as above, acknowledging the connection request and video port

After initiating the connection, all further packets use a standard format:
```c
struct drone_packet {
    u8 packet_magic;    // Always 0xCC
    u16 packet_length;  // Length of entire packet, stored in upper 13 bits (i.e. length << 3)
    u8 header_crc8;     // Fast-CRC8 (seed=119) of the first 3 bytes
    u8 packet_type;     // Purpose currently unknown
    u16 cmd_id;         // Command type, the command packet and its response both have the same ID
    u16 seq_num;        // Packet sequence number, drone's response contains the sequence number
                        // of the command so its response can be matched
    u8 data[];          // Packet data, the format is dependent on the command ID
    u16 packet_crc16;   // Fast-CRC16 (seed=13970) of all packet bytes (except these two bytes)
}
```

### Known Commands IDs
* 18 - Command: Get SSID. Response data is SSID
* 17 - Command: Set SSID[§](#-standard-response). Command data is new SSID
* 19 - Command: Get WIFI Password. Response data is WIFI password
* 20 - Command: Set WIFI Password[§](#-standard-response). Command data is new WIFI password
* 21 - Command: Get Country Code. Response data is country code
* 22 - Command: Set Country Code[§](#-standard-response). Command data is new country code
* 26 - Drone Info: WIFI State. First byte is WIFI strength, second byte is 'WIFI Disturb'
* 32 - Command: Set Bitrate[§](#-standard-response). Command data is one byte of bitrate (meaning values is TODO, 0 when auto?)
* 33 - Command: Set Automatic Bitrate[§](#-standard-response). Command data is one byte, 1 for enabled, 0 for disabled
* 36 - Command: Set Electronic Image Stabilization[§](#-standard-response). Command data is one byte, 1 for enabled, 0 for disabled
* 37 - Command: Produce I-frame/SPS/PPS[§](#-standard-response) (Not clear, but needed to be sent periodically to drive the video stream)
* 40 - Command: Get Bitrate. First byte signifies success (see [§](#-standard-response)), second byte is bitrate (see Set Bitrate)
* 48 - Command: Take a Picture[§](#-standard-response)
* 49 - Command: Set Camera Mode[§](#-standard-response). Command data is one byte, 1 for 'photo', 0 for 'video' (TODO: Effect Unknown)
* 50 - Command: Set Recording[§](#-standard-response). Command data is one byte, 0 for recording
* 52 - Command: Set Camera Exposure Value[§](#-standard-response). Command data is one byte, EV-value in range [-9, 9]
* 53 - Drone Info: Light Strength. Data is one byte, the 'light strength'
* 55 - Command: Set Photo Quality[§](#-standard-response). Command data is one byte, 1 for high quality, 0 for low quality
* 67 - Drone Info: Error Tip 1. Data is error tip string
* 68 - Drone Info: Error Tip 2. Data is error tip string (TODO: What is the difference)
* 69 - Command: Get Firmware Version. Response data is 31 bytes long, first byte is success flag (see [§](#-standard-response))
  last 30 bytes are version string. In actuality, only first 11 bytes are in use (Version format: `xx.xx.xx.xx`)
* 70 - App Query: Get Current Time. App's response data is a success flag (see [§](#-standard-response)) and then 
  ```c
  struct get_current_time_response {
    u16 year;
    u16 month;
    u16 day;
    u16 hour;
    u16 minute;
    u16 second;
    u16 millisecond;
  }
  ```
* 71 - Command: Get Activation Data. First byte of response data is a success flag (see [§](#-standard-response)) and then
  ```c
  struct get_activation_data_response {
    u16 activation_year; // Always 2018
    u16 activation_month;
    u16 activation_day;
    u16 activation_hour;
    u16 activation_minute;
    u16 activation_second;
    u16 activation_millisecond;
    u8 raw_serial_bytes[14]; // App calculates hex md5 digest of these bytes
    
    // Conjecture: This is the manufacturing date
    u8 unknown_flag;
    u16 unknown_year;
    u16 unknown_month;
    u16 unknown_day;
    u16 unknown_hour;
    u16 unknown_minute;
    u16 unknown_second;
    u16 unknown_millisecond;
    u8 unknown_bytes[14];
  }
  ```
* 72 - Command: Get Unique Identifier. First byte of response data is a success flag (see [§](#-standard-response)), then 16 bytes of the unique ID.
* 73 - Command: Get Loader Version. First byte of response data is a success flag (see [§](#-standard-response)) and then 30 bytes of
  version string. Only the first 11 bytes are actually in use. Version format: `aa.bb.xx.xx`, where `aa` is `01` if it's a TelloEDU drone, and `bb` is some unknown flag.
* 74 - Command: Shutdown Drone
* 75 - Command: Get Activation Status. Response data is a single byte, 0 is activated, 1 if not.
* 76 - Command: Activate Drone[§](#-standard-response). Drone data is the activation (current) time, in the follow format:
  ```c
  struct activation_time {
    u16 year;
    u16 month;
    u16 day;
    u16 hour;
    u16 minute;
    u16 second;
    u16 millisecond; 
  }
  ```
* 80 - Command: Set Flight Controls. Sent by the app about 50 times/second. Data format is:
  ```c
  struct flight_controls {
    u8 packed_controls[6]; // UUU|Q|DDDDDDDDDDD|CCCCCCCCCCC|BBBBBBBBBBB|AAAAAAAAAAA
                           // U is unused, Q is a Quick Flight Speed bit, and A, B, C, and D are four
                           // axes of motion, whose range is [364, 1684] such that 1024 is no motion
                           // TODO: Which axis is which
    u8 hour;
    u8 minute;
    u8 second;
    u16 millisecond;
  }
  ```
* 84 - Command: Take Off[§](#-standard-response)
* 85 - Command: Land Drone[§](#-standard-response). Has one byte of command data which is 0 for landing and 1 for canceling an existing landing request.
* 86 - Drone Info: Flight Data. Sent from the drone periodically, has three forms, depending on the packet size:
  ```c
  struct flight_data {
    i16 height; // in decimeters (meters * 10)
    i16 north_speed; // in decimeters/second (meters/second * 10)
    i16 east_speed; // in decimeters/second (meters/second * 10)
    i16 ground_speed;
    i16 flight_time;
    u1 imu_state;
    u1 pressure_state;
    u1 down_visual_state;
    u1 power_state;
    u1 battery_state;
    u1 gravity_state;
    u1 unused_or_unknown;
    u1 wind_state;
    
    // Next fields exist if packet size >= 19
    i8 imu_calibration_state;
    i8 battery_percentage;
    i16 flight_time_left;
    i16 battery_left;
    u1 eMSky;
    u1 eMGround;
    u1 eMOpen;
    u1 drone_hover;
    u1 outage_recording;
    u1 battery_low;
    u1 batery_lower;
    u1 factory_mode;
    u8 flight_mode;
    u8 throw_fly_timer;
    u8 camera_state;
  
    // Next field exists if packet size >= 22
    u8 electrical_machinery_state;
  
    // Next fields exist if packet size >= 23
    u1 front_in;
    u1 front_out;
    u1 front_LSC;
    u2 center_gravity_calibration_status;
    u1 soaring_up_into_the_sky;
    u1 unused_or_unknown;
    u1 temperature_height;
  
    // Next field exists if packet size <= 24
    u8 unused_or_unknown;
  }
  ```
* 92 - Command: Flip Drone[§](#-standard-response). Has one byte of command data, which is the flip direction, range [0, 7] (TODO meaning of values)
* 93 - Command: Throw and Fly[§](#-standard-response)
* 94 - Command: Palm Land[§](#-standard-response)
* 128 - Command: Set Smart Video mode[§](#-standard-response). Has one byte of command data, with the bottom bit being 1 for start and 0 for stop, and the next 2 bits being one of: Rotate 360 - 1, Circle - 2, Up & Away - 3)
* 129 - Drone Info: Smart Video status. Sent from the drone periodically while a smart video mode is enabled. Data format currently unknown.
* 4176 - Drone Info: Drone Log Header[*](#-drone-log)
* 4177 - Drone Info: Drone Log Data[*](#-drone-log)
* 4178 - Drone Info: Drone Log Configuration[*](#-drone-log)
* 4179 - Command: Set bouncing mode[§](#-standard-response). Has one byte of command data, which is 48 for enabling bouncing and 49 for disabling bouncing
* 4181 - Command: Set Low Battery Warning[§](#-standard-response). Packet data is two bytes encoding LE low battery warning level
* 4182 - Command: Get Flight Height Limit. First byte of response data is a success flag (see [§](#-standard-response)) and then two bytes encoding LE flight height limit
* 4183 - Command: Get Low Battery Warning. First byte of response data is a success flag (see [§](#-standard-response)) and then two bytes encoding LE low battery warning level
* 4184 - Command: Set Attitude Angle[§](#-standard-response). Packet data is an LE float (four bytes) encoding the "attitude angle" (TODO meaning)
* 4185 - Command: Get Attitude Angle. First byte of response data is a success flag (see [§](#-standard-response)) and then an LE float (four bytes) encoding the attitude angle

#### § Standard Response
These packets have a standard response, the packet data is a single byte,
where 0 signifies success and any other value is a failure.

#### * Drone Log
The drone has an internal log which it broadcasts to the app. The app just concatenates all log payloads (it does not distinguish between the 
three packet types), and saves the log to a file. The file seems to be a proprietary log format used by many DJI drones, so it was
already reversed, and a tool exists to parse it: [https://datfile.net/index.html](https://datfile.net/index.html). It includes accurate information from the onboard MVO like position and velocity in all axis, and from the onboard IMU like the position and rotation in space (in quaternions) and the temperature.   

### Unknown Command IDs (TODO)
8, 16, 25, 27, 34, 35, 51, 65, 81, 82, 83, 88, 89, 90, 91, 95, 96, 97, 98, 99, 101, 112, 113, 114, 116, 117, 128, 129, 4179, 4180

## Video Socket
This connection is only one-way: The app listens and receives video packets, with
the standard format:
```c
struct video_packet {
    u8 frame_number;    // The number of the current frame 
    u7 segment_number;  // This is a bitfield: The first 7 bits of the second byte are the current
    u1 last_segment;    // segment number and the last bit is a "last segment in frame" flag
    u8 raw_nalu[];      // Raw H264 frame (Network Abstraction Layer Unit)
}
```
Frames are sent in multiple segments, and the app uses the frame number and segment number to
reconstruct the frames and detect missing frames/segments. The raw NALU is passed to a native
android H264 decoder, or in our case to FFMPEG.
