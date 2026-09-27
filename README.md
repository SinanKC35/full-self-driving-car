🚗 Full Self-Driving Car Simulation

A C++ Object-Oriented Programming project that simulates the behavior and navigation of a fully autonomous vehicle inside a 2D grid world.

The simulation demonstrates core Object-Oriented Programming (OOP) concepts such as:

Encapsulation
Inheritance
Polymorphism
Composition
Object-to-object communication
Dynamic memory management

The autonomous vehicle uses multiple sensors to perceive its environment, combines the sensor measurements through a Sensor Fusion Engine, and uses a Navigation System to follow predefined GPS targets.

📌 Project Overview

The simulation creates a two-dimensional GridWorld containing:

🚗 Moving cars
🚲 Moving bikes
🅿️ Parked cars
🛑 Stop signs
🚦 Traffic lights
🤖 One autonomous Self-Driving Car

The objects are placed randomly in the world using a configurable random seed.

The Self-Driving Car continuously:

Collects information from its sensors.
Combines the sensor measurements.
Evaluates obstacles and traffic lights.
Determines the appropriate direction.
Accelerates or decelerates when necessary.
Moves toward its GPS destination.
Stops when all GPS targets are reached or when it leaves the world boundaries.
🧱 Main Components
GridWorld

The GridWorld represents the simulation environment.

It is responsible for:

Storing world objects.
Updating objects on every simulation tick.
Detecting objects outside the world boundaries.
Displaying the complete world.
Displaying the area around the autonomous vehicle.
🚘 SelfDrivingCar

The autonomous vehicle contains:

A NavigationSystem
Multiple sensors
Position
Direction
Speed state

The vehicle can have three speed states:

STOPPED
HALF_SPEED
FULL_SPEED

The vehicle changes its behavior according to sensor information and its current GPS target.

🛰️ Sensors

The Self-Driving Car uses three different sensors.

Lidar
Range: 9 cells
360° field of view
Detects static and moving objects
Provides distance, object category and confidence
Radar
Range: 12 cells
Detects moving objects
Focuses on objects directly in front of the vehicle
Provides distance, speed, direction and confidence
Camera
Range: 7 cells
Detects objects in front of the vehicle
Can detect traffic-light colors
Can detect traffic-sign text
Provides detailed object information

Each sensor produces SensorReading objects containing information such as:

Object ID
Position
Object Type
Distance
Confidence
Speed
Direction
Traffic Light Color
Sign Text
🧠 Sensor Fusion

The SensorFusionEngine combines measurements coming from different sensors.

Measurements referring to the same object are grouped using the object's ID.

A weighted average is used for distance measurements, where readings with higher confidence have greater influence.

Low-confidence measurements can be rejected using:

--minConfidenceThreshold

There is also a special safety rule for bikes: if at least one sensor detects a bike, the measurement is not discarded because of the confidence threshold.

🧭 Navigation System

The NavigationSystem receives a list of GPS targets.

Example:

--gps 10 10 15 15

The vehicle follows the targets sequentially.

The distance between two positions is calculated using Manhattan distance:

|x1 - x2| + |y1 - y2|

The navigation system determines whether the vehicle should move:

NORTH
SOUTH
EAST
WEST

It also decides when the vehicle should:

Accelerate
Decelerate
Stop
Turn
🚦 Autonomous Driving Rules

The vehicle reacts to its environment.

Traffic Lights

The vehicle stops when it detects a:

RED light
YELLOW light

within 3 cells.

Moving Obstacles

The vehicle stops when a moving:

Car
Bike

is detected within 2 cells.

Stop Signs

The vehicle stops when a STOP sign is detected within 1 cell.

GPS Target

The vehicle slows down when it is within 5 cells of its current GPS target.
🏗️ Object-Oriented Design

The project uses an inheritance hierarchy for the objects in the world.

WorldObject
│
├── StaticObject
│   ├── TrafficLight
│   ├── StopSign
│   └── StationaryVehicle
│
└── MovingObject
    ├── Car
    ├── Bike
    └── SelfDrivingCar

The sensor hierarchy is:

Sensor
│
├── LidarSensor
├── RadarSensor
└── CameraSensor

Virtual functions are used to demonstrate polymorphism, while classes such as SelfDrivingCar use composition by containing a NavigationSystem and a collection of sensors.

🗺️ Visualization

The program provides two types of visualization.

Full World Visualization

Displays the complete grid at the beginning and end of the simulation.

POV Visualization

Displays the area around the Self-Driving Car after every simulation tick.

Symbols
Symbol	Meaning
@	Self-Driving Car
.	Empty cell
X	Outside world bounds
R	Red traffic light
Y	Yellow traffic light
G	Green traffic light
S	Stop sign
B	Moving bike
C	Moving car
P	Parked car
?	Unknown object
⚙️ Requirements

The project requires:

C++ compiler
g++
GNU Make
C++11 or newer

The provided Makefile uses:

-Wall
-std=c++11
-O2
🔨 Compilation

From the project directory, run:

make

This creates the executable:

simulation

To remove the compiled files:

make clean
▶️ Running the Simulation

The basic syntax is:

./simulation --gps <x1> <y1> <x2> <y2> ...

Example:

./simulation --seed 12 --dimY 20 --simulationTicks 10 --gps 10 10 15 15

This example:

Uses random seed 12
Creates a world of 40 × 20
Runs for a maximum of 10 simulation ticks
Uses (10,10) as the first GPS target
Uses (15,15) as the second GPS target
🛠️ Available Parameters
--seed <N>
--dimX <N>
--dimY <N>
--numMovingCars <N>
--numMovingBikes <N>
--numParkedCars <N>
--numStopSigns <N>
--numTrafficLights <N>
--simulationTicks <N>
--minConfidenceThreshold <N>
--gps <x1> <y1> <x2> <y2> ...
--help

For example:

./simulation --help
🧪 Example Execution

The following command was used to test the project:

make clean && make
./simulation --seed 12 --dimY 20 --simulationTicks 10 --gps 10 10 15 15

The program compiled successfully using:

g++ -Wall -std=c++11 -O2

During the execution, the Self-Driving Car was initialized at:

(17, 14)

with the first GPS target:

(10, 10)

The simulation demonstrated sensor fusion and autonomous decisions.

For example, the vehicle detected a moving bike and stopped:

Sensor Fusion found 2 objects.
WARNING: Stopping for Moving obstacle (BIKE:2)
Car moved to (17, 14) at speed 0 facing 0

After the obstacle was no longer blocking its movement, the vehicle changed direction and accelerated:

ACTION: Turning to new direction: 3
ACTION: Accelerating.
Car moved to (16, 14) at speed 1 facing 3

It subsequently continued moving and reacting to detected objects:

ACTION: Accelerating.
Car moved to (14, 14) at speed 2 facing 3

WARNING: Stopping for Moving obstacle (BIKE:2)
Car moved to (14, 14) at speed 0 facing 3

The simulation completed normally after the configured 10 ticks:

*** SIMULATION FINISHED ***
📊 Example Result

For the test above, the vehicle demonstrated:

Successful compilation
GPS-based navigation
Direction changes
Acceleration
Deceleration/stopping
Moving-obstacle detection
Sensor fusion
Grid visualization
Autonomous movement

The final visualization showed the state of the 40 × 20 GridWorld after the simulation.

📁 Project Structure
full self-driving car/
│
├── main.cpp
├── Simulation.cpp
├── Simulation.h
├── Makefile
└── README.md
main.cpp

Handles:

Command-line arguments
Simulation configuration
GPS targets
World initialization
Main simulation loop
Simulation.h

Contains the declarations of the main classes, structures and enumerations.

Simulation.cpp

Contains the implementation of:

World objects
Sensors
Sensor fusion
Navigation
Self-driving behavior
Grid visualization
Makefile

Automates compilation and cleaning of the project.

🎯 Conclusion

This project provides a simplified simulation of an autonomous vehicle using C++ and Object-Oriented Programming principles.

The combination of sensors, sensor fusion, GPS navigation and autonomous decision-making creates a small-scale representation of the main components involved in autonomous driving systems.
