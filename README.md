#ESPNOW Communication Module for 2 ESP32 Microcontrollers (1 DEV MODULE, 1 WROVER MODULE with CAM) powered by the same breadboard. 

Board A Sender MAC: A0:B7:65:21:EE:9C
Board B Receiver MAC: E0:8C:FE:F5:7E:64

Board A: (LLEAP_TestBend_Sender.ino) sends test trajectories of 45, 90, and 15 degrees and simulates fault injection.
Board B: (LLEAP_Joint_Controller.ino) includes PID controller calcs. 

