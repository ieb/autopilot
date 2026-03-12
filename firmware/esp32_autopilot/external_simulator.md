# External Simulation

The aim is to simulate as much of the ESP32 autopilot code as possible, as close to the real installation in the boat as possible. The current hal_sim code internally simulates the environment of the boat. I want to move this out to an external program communicating over sockets with the ESP32 hal_sim code.

I want to implment extrnal simulation for the ESP32 hal simulator over a TCP socket. The simulation will write and read messages containing PGN messages to the Socket and will use a special PGN message simulate the IMU readings which will then update th BNO055 registers.

data flow is

Python CAN Bus Simulator (Tx) -- CAN_socket --> CAN Rx Queue
Python CAN Bus Simulator (Rx) <-- CAN_socket -- CAN Tx Queue 
Python IMU Simulator -- IMU_socket --> BNO055 registers

CAN_socket and IMU_socket can be different sockets, but if the message flows can be accomidate on the same socket that would be better.

The ESP32 will emit Rudder possition PGN messages as well as pitch and roll PGN messages.

The Python code will be driven by the boat simulator which can be found here /Users/boston/ieb/autopilot/src/simulation/yacht_dynamics.py


