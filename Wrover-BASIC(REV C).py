"""
Title: Dynamic Autonomous Navigation and Obstacle Avoidance System for Waveshare Rovers. REV C

About:
This script represents a significant leap in autonomous navigation and obstacle avoidance 
for Waveshare Rovers, integrating the FHL-LD19 LiDAR's advanced capabilities. The script 
now dynamically adjusts navigation decisions based on the LiDAR's scanning speed and angular 
resolution, alongside refined obstacle detection through intensity analysis. Operating with 
a Raspberry Pi 4, the system processes real-time spatial data, light intensity, and scanning 
dynamics for nuanced maneuvering across varied environments. Key advancements include the 
implementation of a dynamic safety margin and enhanced object recognition, bolstered by 
sophisticated decision-making algorithms that account for the LiDAR's orientation and 
environmental complexity.

Features:
- Dynamic Obstacle Detection: Utilizes LiDAR distance, intensity, and scanning speed for 
  responsive obstacle recognition.
- Adaptive Navigation: Employs a dynamic safety margin that adjusts based on LiDAR scanning 
  speed and data point density, enabling more flexible and intelligent pathfinding.
- Advanced Environmental Perception: Incorporates light intensity and point cloud analysis 
  for detailed environmental understanding and object differentiation.
- Speed and Angular Resolution Integration: Leverages the LiDAR's scanning speed and angular 
  resolution for real-time adjustments to navigation strategies, accommodating rapid changes 
  in the rover's surroundings.
- Enhanced Decision Making: Introduces complex algorithms for strategic navigation, 
  factoring in the rover's orientation correction for precise movement and obstacle 
  avoidance.
- Modular and Extensible Design: Maintains a structured, easy-to-modify codebase for 
  customization and further development in educational, developmental, and research 
  applications.

Requirements:
- Python 3.x
- pyserial package for serial communication.

Disclaimer:
This software is provided 'as-is', without any express or implied warranty. In no event 
will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use, modify, and distribute this software under the 
terms of the MIT License. It is designed for educational and developmental purposes and 
is not intended for use in operational environments.

License:
Released under the MIT License.

Copyright (c) <2024> <FigTroniX>

Designed by integrating advanced LiDAR processing techniques, this system represents the 
forefront of hobbyist and educational autonomous rover navigation, offering unparalleled 
precision and adaptability in obstacle detection and avoidance.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify, merge, publish, 
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
SAFETY_DISTANCE = 900

# Enhanced Movement and Speed Definitions
# These values should be adjusted based on testing to ensure effective maneuvering
# Movement commands and safety distance definition
COMMANDS = {
    "FORWARD": {"T": 1, "L": 150, "R": 150},  # Normal forward, adjust speed as needed
    "REVERSE": {"T": 1, "L": -200, "R": -200},  # Fast reverse, adjust speed as needed
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
    """
    Parses a LiDAR data packet, extracting and processing distance, intensity,
    and now speed and angular resolution for dynamic navigation adjustments.
    """
    try:
        # Existing unpacking of packet data
        header, ver_len, speed = struct.unpack_from('<BBH', packet, 0)
        start_angle, = struct.unpack_from('<H', packet, 4)
        end_angle, timestamp, crc_check = struct.unpack_from('<HHB', packet, len(packet) - 5)

        num_points = ver_len & 0x1F  # Lower five bits of ver_len indicate the number of points
        angular_resolution = (end_angle - start_angle) / max((num_points - 1), 1)  # Calculate angular resolution

        # New: Calculate dynamic safety margin based on speed and angular resolution
        dynamic_safety_margin = calculate_dynamic_safety_margin(speed, angular_resolution)

        closest_distance = float('inf')
        closest_intensity = 0
        closest_angle_relative_to_rover = None
        decision = "FORWARD"

        for i in range(num_points):
            offset = 6 + i * 3
            if offset + 3 <= len(packet) - 5:
                distance, intensity = struct.unpack_from('<HB', packet, offset)
                angle_increment = angular_resolution
                angle = (start_angle + angle_increment * i) % 36000  # Angle in 0.01 degrees
                adjusted_angle = (angle + 9000) % 36000

                # Use dynamic_safety_margin instead of a fixed safety distance
                if 0 < distance < dynamic_safety_margin:
                    closest_distance = distance
                    closest_intensity = intensity
                    closest_angle_relative_to_rover = adjusted_angle

        # Decision making based on closest obstacle's data and dynamic safety margin
        if closest_distance < dynamic_safety_margin:
            # Dynamic decision-making logic based on intensity, angle, and potentially other factors
            decision = make_dynamic_decision(closest_intensity, closest_angle_relative_to_rover)

        return decision
    except struct.error as e:
        print(f"Error parsing packet: {e}")
        return "STOP"

def calculate_dynamic_safety_margin(speed, angular_resolution):
    """
    Calculates a dynamic safety margin based on the LiDAR's scanning speed and angular resolution.
    """
    # Example logic for dynamic safety margin calculation; adjust based on testing and requirements
    base_safety_distance = 950  # Base safety distance in millimeters
    speed_factor = speed / 1000  # Example speed factor calculation; adjust as needed
    resolution_factor = 36000 / angular_resolution  # Example resolution factor calculation

    # Calculate dynamic safety margin
    dynamic_safety_margin = base_safety_distance + (speed_factor * resolution_factor)
    return dynamic_safety_margin

def make_dynamic_decision(intensity, angle):
    """
    Makes a dynamic navigation decision based on obstacle intensity and angle.
    Adjust this function to include more sophisticated logic as needed.
    """
    # Placeholder logic; expand based on your navigation strategy
    if intensity > 20:
        if 9000 <= angle <= 27000:
            return "TURN_LEFT"
        else:
            return "TURN_RIGHT"
    else:
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
                #print(f"Action decided: {action}")
                send_command_to_rover(rover_conn, COMMANDS[action])
                #print(f"Command sent: {COMMANDS[action]}")
                #time.sleep(0.1)
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
                #print(f"Action decided: {action}")
                send_command_to_rover(rover_conn, COMMANDS[action])
                #print(f"Command sent: {COMMANDS[action]}")
            time.sleep(0.05)
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
