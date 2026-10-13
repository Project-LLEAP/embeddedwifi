ESP-NOW Joint Comms Module 

Overview: 

Implement wireless ESP-NOW communication between a main ESP32 controller and distributed ESP32 joint controllers for LLEAP exoskeleton. The main controller generates the leg motion profile, converts task-space motion into joint-space commands, and sends appropriate command data to the ESP32 assigned to each hip or knee joint. Each joint ESP32 runs its own local PID controller and drives its own ESCON motor controller. The PID loop should be independent from ESP-NOW processing. Incoming packets are validated and copied by the ESP-NOW callback, while the main joint-control loop handles the resulting command data at its normal control rate. 

System Architecture: 
Brain ESP32 > ESP-NOW > Joint ESP32 > ESCON Control > Motor > Mechanical Leg Joint 
- Main ESP32: Generate motion profiles, inverse kinematics calculations, and sends joint-command data. 
- Joint ESP32: Receives command data, runs local encoder feedback and PID position control, and commands its associated ESCON and motor.
- ESP-NOW: Low-latency wireless comms without requiring a Wi-Fi access point.

Note: Encoder feeds back to joint ESP32 for closed loop control. 

Board A is the "brain" board. Board B is the joint-controller test board for one hip or knee actuator at a time. 

Old Code Files: 
- PID_ESPNOW_Integration.ino: Joint-controller firmward to run encoder, PID loop, ESCON outputs, serial interface, and comms integration.
- BrainSender_V2_Test.ino: Bench-test firmward for main-controller role. Send V2 trajectory data packets to one "joint" ESP32. 

Shared Comms library: 
- JointComms.h: Defines the shared ESP-NOW protocol, trajectory-data, fault type, and JointComms class.
- JointComms.cpp: ESP-NOW Init, peer setup, packet validation, data handoff, and E-STOP latch.
- LLEAP_Joint_Controller.ino: Board B PID/Encoder/ESCON code
- LLEAP_TestBench_Sender.ino: Board A Test-Brain Code

Data Packets: Contains protocol identifier, version #, packet type, sequence number, and trajectory-command values such as initial/final position/velocity and duration. Each receiver rejects packets with unmatched length, protocol identifier, or version. Heartbeat packets and motion commands are different packet types. Included are a source-MAC validation, 250ms radio, reset fault latching, safe enable conditions. 

Safety: 
Valid FAULT packets causes the receiving joint controller to 
1. Drive ESCON enable/inhibit output low. 
2. Command zero DAC effort. 
3. Latch E-STOP. 
4. Ignore later normal command packets. 
5. Block serial motor-enable commands until joint ESP32 reset. 

Note: PID calcs, serial output, and normal command handling are outside callback.

Bech-Test Procedure: 
1. Check that Brain Sender and Joint Controller ESP32 boards have the correct MAC addresses. Also check ESCON polarity, direction polarity, DAC scaling/limit, joint limits. 
2. Power boards independently on the same ESP-NOW Wi-Fi channel and flash to both boards. 
3. Confirm Joint Controller prints correct trajectory-data packets.
4. Send fault packet and verify joint controller latches E-STOP. 
5. Before connecting motor, verify ESCON disable and zero-command signals with motor power off. 

Current Test Bench Setup: 

#ESPNOW Communication Module
#2 ESP32 Microcontrollers (1 DEV MODULE, 1 WROVER MODULE with CAM) powered by the same breadboard using VIN pin. 

Board A Sender MAC: A0:B7:65:21:EE:9C
Board B Receiver MAC: E0:8C:FE:F5:7E:64

Board A: (LLEAP_TestBend_Sender.ino) sends test trajectories of 45, 90, and 15 degrees and simulates fault injection.
Board B: (LLEAP_Joint_Controller.ino) includes PID controller calcs. 
JointComms.h file establishes Sender-Receiver communication line. Defines the shared ESP-NOW protocol, trajectory-data structures, fault type, and JointComms class interface. 

Current Status and Limitations
Two-board ESP-NOW packet transder, validated trajectory-data reception, and latched software/hardware-output E-STOP path. Still need motor-level fault latency, loop-jitter measurements, comms-loss timeout, and final multi-joint integration. 
