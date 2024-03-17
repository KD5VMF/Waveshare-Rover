"""
Title: Advanced Rover Obstacle Avoidance System

About:
This Python script embodies an advanced navigation and obstacle avoidance system,
specifically designed for rovers equipped with LiDAR (FHL-LD19) sensor. It establishes
and manages serial connections with the Waveshare rover and its LiDAR sensor, adeptly
processing real-time LiDAR data to intelligently detect and navigate around obstacles.
Using a Raspberry Pi 4 as the main controller, key features include dynamic obstacle
detection, decision-making based on sensor input, and executing movement commands
tailored to the rover's context. Designed with robustness in mind, it includes
comprehensive error handling and is structured into clear sections for configuration,
utility functions, LiDAR processing, and main logic.

Requirements:
- Python 3.x
- pyserial package for serial communication

Disclaimer:
This software is provided 'as-is', without any express or implied warranty. In no event
will the authors be held liable for any damages arising from the use of this software.
You are permitted to use, modify, and distribute this software under the terms of the
MIT License. This software is intended for educational and developmental purposes; the
creators assume no responsibility for its use in operational environments. By using
this software, you agree to the terms of this disclaimer and the accompanying license
agreement.

License:
This software is released under the MIT License.

MIT License

Copyright (c) <year> <copyright holders>

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
OR OTHER DEALINGS IN THE SOFTWARE.
"""

import serial
import struct
import json
import time

# ================================================
# Configuration and Constants
# ================================================

# Rover connection settings
ROVER_SERIAL_PORT = '/dev/serial0'
ROVER_BAUD_RATE = 1000000

# Lidar connection settings
LIDAR_SERIAL_PORT = '/dev/ttyUSB0'
LIDAR_BAUD_RATE = 230400

# Define a safety distance (in millimeters)
SAFETY_DISTANCE = 800

# Enhanced Movement and Speed Definitions
# These values should be adjusted based on testing to ensure effective maneuvering
# Movement commands and safety distance definition
COMMANDS = {
    "FORWARD": {"T": 1, "L": 100, "R": 100,  # Normal forward, adjust speed as needed
    "REVERSE": {"T": 1, "L": -125, "R": -125},  # Fast reverse, adjust speed as needed
    "TURN_LEFT": {"T": 1, "L": -255, "R": 255},  # Sharp left turn, negative value for left motor, positive for right
    "TURN_RIGHT": {"T": 1, "L": 255, "R": -255},  # Sharp right turn, positive value for left motor, negative for right
    "STOP": {"T": 0}  # Emergency stop
}

# ================================================
# Utility Functions
# ================================================

def open_serial_connection(port, baud_rate):
    """Attempts to open a serial connection to a given port with specified baud rate."""
    try:
        return serial.Serial(port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}")
        return None

def send_command_to_rover(serial_conn, command):
    """Sends a JSON-formatted command to the rover over a serial connection."""
    command_str = json.dumps(command) + '\n'
    try:
        serial_conn.write(command_str.encode())
    except serial.SerialException as e:
        print(f"Failed to send command to rover: {e}")

# ================================================
# LiDAR Processing
# ================================================

def CalCRC8(data):
    """Calculate the CRC8 for the given data using the CrcTable."""
    # CRC8 table for LD19 LiDAR data validation, predefined as per device specifications.
    CrcTable = [
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x3a, 0x77, 0xa0, 0xed, 0x43, 0x0e, 0xd9, 0x94, 0xc8, 0x85, 0x52, 0x1f, 0xb1, 0xfc, 0x2b, 0x66,
        0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
        0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
        0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
        0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
        0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
        0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
    ]
    crc = 0
    for byte in data:
        crc = CrcTable[(crc ^ byte) & 0xFF]
    return crc
def find_packet_start(serial_port):
    """Read bytes one at a time until the packet start header (0x54) is found."""
    while True:
        byte = serial_port.read(1)
        if byte == b'\x54':  # Header byte for packet start
            return byte  # Return the header to be included in the packet
            
def read_packet(serial_port):
    """Read a complete packet dynamically, based on header detection."""
    packet = find_packet_start(serial_port)  # Include the packet start
    packet += serial_port.read(21)  # Example: Read the next 21 bytes; adjust based on your packet structure
    return packet

def parse_lidar_packet(packet):
    """Parse the LiDAR packet and extract data for obstacle detection, considering the LiDAR's orientation."""
    try:
        header, ver_len, speed, start_angle = struct.unpack_from('<BBHH', packet, 0)
        end_angle, timestamp, crc_check = struct.unpack_from('<HHB', packet, len(packet) - 5)
        num_points = ver_len & 0x1F  # Extract the number of points from ver_len

        # Debugging prints
        #print(f"Start angle: {start_angle}, End angle: {end_angle}, Num points: {num_points}")

        # Validate num_points to avoid division by zero
        if num_points <= 1:
            print("Error: num_points is less than or equal to 1, which may lead to division by zero.")
            return "STOP"

        # Initialize variables to track the closest obstacle
        closest_distance = float('inf')
        closest_angle = None

        for i in range(num_points):
            offset = 6 + i * 3
            if offset + 3 <= len(packet) - 5:
                distance, angle_code = struct.unpack_from('<HB', packet, offset)
                angle_increment = (end_angle - start_angle) / max((num_points - 1), 1)  # Avoid division by zero
                angle = (start_angle + i * angle_increment) % 360
                
                # Adjust for LiDAR's orientation being at 9 o'clock (270 degrees offset from rover's 12 o'clock)
                # This effectively rotates the LiDAR's "forward" to align with the rover's forward direction
                adjusted_angle = (angle + 90) % 360  # Adding 90 degrees to adjust for the 9 o'clock orientation

                if 0 < distance < closest_distance:
                    closest_distance = distance
                    closest_angle = adjusted_angle

        if closest_distance < SAFETY_DISTANCE:
            print(f"Closest obstacle at {closest_distance}mm, adjusted angle {closest_angle} degrees.")
            # Adjust decision logic based on the LiDAR's adjusted orientation
            if 45 <= closest_angle <= 135:
                return "TURN_RIGHT"  # Adjusted actions based on LiDAR's orientation
            elif 225 <= closest_angle <= 315:
                return "TURN_LEFT"  # Adjusted actions based on LiDAR's orientation
            else:
                return "REVERSE"
        return "FORWARD"
    except struct.error as e:
        print(f"Error parsing packet: {e}")
        return "STOP"

def determine_dynamic_action(closest_angle, lidar_data):
    """
    Decides whether to reverse, turn left, or turn right based on the direction with the furthest distance
    from obstacles. This function assumes access to more comprehensive LiDAR data to make this decision.
    
    Parameters:
    - closest_angle: The angle of the closest detected obstacle.
    - lidar_data: A collection of distances measured by the LiDAR, indexed by their angle.
    
    Returns:
    - A string indicating the chosen action ("REVERSE", "TURN_LEFT", or "TURN_RIGHT").
    """
    
    # Sample angles for left, right, and rear directions
    # These values should be adjusted based on the specific orientation and FOV of your LiDAR
    left_angle = (closest_angle + 90) % 360
    right_angle = (closest_angle - 90) % 360
    rear_angle = (closest_angle + 180) % 360

    # Retrieve the distances for left, right, and directly behind the rover
    left_distance = lidar_data.get(left_angle, 0)
    right_distance = lidar_data.get(right_angle, 0)
    rear_distance = lidar_data.get(rear_angle, 0)

    # Determine the safest direction based on available distances
    if left_distance > right_distance and left_distance > rear_distance:
        return "TURN_LEFT"
    elif right_distance > left_distance and right_distance > rear_distance:
        return "TURN_RIGHT"
    else:
        # If reversing has more space or if left/right distances are too close, choose to reverse
        # Additional logic can be added here to handle cases where all directions are blocked
        return "REVERSE"


def check_for_obstacles(lidar_conn):
    packet = read_packet(lidar_conn)
    if packet:
        return parse_lidar_packet(packet)
    return False


def lidar_processing_thread(lidar_conn, action_queue):
    """Processes LiDAR data in a separate thread, detecting obstacles and deciding movement actions."""
    while True:
        try:
            action = check_for_obstacles(lidar_conn)
            if action:
                action_queue.put(action)
        except Exception as e:
            print(f"Error in LiDAR processing: {e}")

def rover_control_thread(rover_conn, action_queue):
    """Controls the rover based on actions decided by the LiDAR processing thread."""
    while True:
        try:
            if not action_queue.empty():
                action = action_queue.get()
                print(f"Action decided: {action}")
                send_command_to_rover(rover_conn, COMMANDS[action])
                print(f"Command sent: {COMMANDS[action]}")
                time.sleep(0.1)
        except Exception as e:
            print(f"Error in rover control: {e}")


# ================================================
# Main Logic
# ================================================

def main():
    """Main function to initialize connections and control the rover based on LiDAR data."""
    rover_conn = open_serial_connection(ROVER_SERIAL_PORT, ROVER_BAUD_RATE)
    lidar_conn = open_serial_connection(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE)
    
    if not rover_conn or not lidar_conn:
        print("Unable to open serial connections. Exiting.")
        return

    try:
        while True:
            action = check_for_obstacles(lidar_conn)
            if action:
                print(f"Action decided: {action}")
                send_command_to_rover(rover_conn, COMMANDS[action])
                print(f"Command sent: {COMMANDS[action]}")
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if rover_conn:
            rover_conn.close()
        if lidar_conn:
            lidar_conn.close()
        print("Connections closed.")

if __name__ == "__main__":
    main()
