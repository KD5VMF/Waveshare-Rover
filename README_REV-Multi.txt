# Enhanced Dynamic Autonomous Navigation and Obstacle Avoidance System (REV-Multi) for Waveshare Rovers

## Overview
The REV-Multi script is designed to revolutionize autonomous navigation and obstacle avoidance for Waveshare Rovers. Utilizing advanced LiDAR processing technology, and optimized for the Raspberry Pi 4, this system introduces sophisticated dynamic safety protocols and enhanced object identification capabilities. Key enhancements include real-time adjustments based on angular resolution, enabling precise maneuvering across varied terrains.

## Multithreading Implementation
The system leverages Python's multithreading capabilities to ensure real-time processing of LiDAR data and responsive rover control. By utilizing separate threads for obstacle detection and rover commands, coupled with a thread-safe action queue, our program achieves efficient parallel execution, enhancing the rover's navigational capabilities in dynamic environments.

## Features
- **Dynamic Obstacle Recognition**: Thoroughly identifies obstacles by analyzing distance, reflectivity, and scan dynamics using the FHL-LD19 LiDAR.
- **Smart Pathfinding**: Dynamically adjusts safety margins in response to LiDAR scan velocity and data density for intelligent route determination.
- **Refined Environmental Awareness**: Achieves a nuanced understanding of surroundings through light intensity and spatial distributions.
- **Angular Resolution Adaptation**: Employs LiDAR's angular resolution data for immediate navigation strategy adjustments.
- **Sophisticated Navigation Algorithms**: Considers the rover's orientation and environmental complexity for precise obstacle navigation.
- **Flexible and Structured Design**: Designed with modularity in mind, allowing for easy adjustments and enhancements.

## System Requirements
- Compatible with Python 3.x environments.
- Requires the `pyserial` library for serial communication between the Raspberry Pi 4 and LiDAR hardware.

## Installation
1. Ensure Python 3.x is installed on your Raspberry Pi 4.
2. Install the `pyserial` package using pip:

pip install pyserial


## Usage
To run the script, navigate to the directory containing the script and execute:

python Wrover(Rev-Multi).py

This command initiates the autonomous navigation and obstacle avoidance system.

## Disclaimer
This software is provided 'as-is', without any warranty. The authors will not be held liable for any damages arising from its use.

## License
Released under the MIT License. See the LICENSE file for more details.

## Contribution
Contributions are welcome! Please fork the repository and submit a pull request with your proposed changes.

## Acknowledgments
Special thanks to the developers and contributors of the FHL-LD19 LiDAR unit and Waveshare Rovers for providing the hardware that inspired this project.

