NOVA
====

NOVA is a custom-built experimental drone platform focused on autonomous flight, real-time sensing, obstacle avoidance, embedded systems, and a custom remote-control interface.

The project is developed from the ground up, including the drone's control electronics, sensor system, communication system, remote controller, display system, and graphics engine.


OVERVIEW
--------

NOVA is designed around a modular architecture consisting of:

- Dual Arduino controllers
- Flight-control system
- Five VL53L0X ToF distance sensors
- Wireless communication
- Custom NOVA-X Remote
- Custom NOVA-X Remote OS
- 2.4-inch 320×240 resistive touchscreen
- Custom display/graphics system
- SD card (in 1-bit) as external storage 


FLIGHT CONTROL
--------------

NOVA uses a dual-Arduino architecture.

The controllers divide the responsibilities of the drone system, allowing flight-critical operations and supporting functions to be handled separately.

The flight-control system is responsible for:

- Flight control
- Motor control
- Stabilization
- Sensor processing
- Obstacle detection
- Safety logic
- Flight-mode management


OBSTACLE DETECTION
------------------

NOVA uses five VL53L0X Time-of-Flight distance sensors for real-time obstacle detection.

The sensors continuously measure distances around the drone and provide information to the flight-control system.

The obstacle-detection system can restrict movement when an obstacle is detected in the intended direction of travel.

Basic control flow:

Joystick command
       |
       v
Flight controller
       |
       v
Distance measurement
       |
       v
Safety check
    /     \
 Safe    Obstacle
  |          |
  v          v
Move       Block


ACTIVE FLIGHT MODE
------------------

Active Flight is the primary manually controlled flight mode.

The pilot controls NOVA through the joystick on the NOVA-X Remote.

Movement commands are processed by the flight controller and checked against the obstacle-detection system before movement is permitted.

This allows the drone to reject or restrict movement commands that would cause it to approach a detected obstacle.


STATIC MODE
-----------

Static Mode is designed for indoor operation.

The objective of Static Mode is to allow NOVA to maintain its position using sensor feedback instead of requiring continuous manual correction.

Static Mode is also intended as a foundation for future position-holding and autonomous-flight functionality.


NOVA-X REMOTE
-------------

NOVA-X is the custom remote-control system developed specifically for NOVA.

The remote provides:

- Joystick-based control
- Touchscreen interaction
- Flight-mode selection
- Real-time telemetry
- Distance-sensor information
- Battery information
- Connection status
- System information
- Warnings and status information


NOVA-X REMOTE OS
----------------

The NOVA-X Remote runs a custom interface called NOVA-X Remote OS.

The operating interface is designed specifically for the hardware of the remote.

The interface is intended to provide the pilot with flight controls and real-time information from the drone through a single integrated system.


DISPLAY
-------

The NOVA-X Remote uses a 2.4-inch resistive touchscreen with a resolution of 320×240 pixels.

The display uses an 8-bit parallel interface.

The display system is being developed with a dedicated graphics architecture to reduce the workload placed on the main controller.


DISPLAY CO-PROCESSOR
--------------------

NOVA-X uses a dedicated display co-processor concept between the main controller and the display.

The purpose of the display co-processor is to handle display-related operations independently and reduce the numbers of IO pins used from the main controller.

The graphics system uses shift registers and dedicated display-data handling to improve the rendering pipeline.


GRAPHICS ENGINE
---------------

The NOVA-X graphics engine is a custom embedded graphics system.

It is designed to handle:

- Full-screen rendering
- UI screens
- Images
- Icons
- Fonts
- Animations
- Boot animations
- Touchscreen interfaces

The current prototype has achieved approximately 5 FPS for full-screen rendering at 320×240 resolution.


EXTERNAL FLASH STORAGE
----------------------

NOVA-X is designed to use external SD card (1-bit formate) for storing graphics and other interface assets.


Capacity:

8 GB

The external flash can be used for:

- UI graphics
- Images
- Animation frames
- Fonts
- Icons
- Boot animations
- Other interface assets


BOOT ANIMATION
--------------

The NOVA-X Remote OS includes a custom boot-animation system.

The planned boot animation contains approximately 192 frames.

Each frame is designed for the 320×240 display resolution.

The animation frames can be stored in external storage and read by the graphics system during startup.


COMMUNICATION
-------------

NOVA and the NOVA-X Remote communicate wirelessly.

The communication system is used for both control and telemetry.

Remote-to-NOVA communication includes:

- Joystick commands
- Flight-mode commands
- Control commands
- Configuration information

NOVA-to-Remote communication includes:

- Distance measurements
- Battery information
- Flight status
- System status
- Telemetry
- Warnings


TELEMETRY
---------

NOVA is designed to provide real-time telemetry to the NOVA-X Remote.

Telemetry can include:

- Battery status
- Distance measurements
- Current flight mode
- Connection status
- Sensor status
- System status
- Warning conditions


SYSTEM ARCHITECTURE
-------------------

The overall NOVA architecture can be represented as:

                         NOVA-X REMOTE
                              |
                       Wireless Link
                              |
                              v
                         NOVA DRONE
                              |
             +----------------+----------------+
             |                |                |
             v                v                v
        Flight Control    Sensors          Telemetry
             |                |
             |           VL53L0X x5
             |
             v
           Motors


HARDWARE
--------

NOVA Drone:

- Arduino-based flight controllers
- Dual-controller architecture
- VL53L0X ToF distance sensors
- Flight sensors
- coreless motors
- LiPo battery
- Wireless communication hardware


NOVA-X Remote:

- Microcontroller
- Joystick
- 2.4-inch 320×240 resistive touchscreen
- External SPI Flash
- Display co-processor
- Shift registers
- Wireless communication hardware


PROJECT GOALS
------------

The long-term goals of NOVA include:

- Reliable manual flight
- Real-time obstacle avoidance
- Stable indoor position holding
- Advanced sensor fusion
- Autonomous navigation
- Autonomous flight
- Improved telemetry
- Advanced NOVA-X Remote OS
- Faster custom graphics rendering
- More capable onboard sensing
- Computer-vision integration
- Autonomous path planning


CURRENT DEVELOPMENT
-------------------

NOVA is an active experimental project.

The hardware, firmware, communication system, flight-control algorithms, sensing system, remote operating system, and graphics engine are continuously being developed and improved.


SAFETY
------

NOVA is an experimental flying platform.

Testing should be performed in a controlled environment with appropriate safety precautions.

The drone should not be operated near people, animals, vehicles, or other hazards.


NOVA
====

Sense.
Control.
Fly.
