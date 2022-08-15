# New Code User Guide

## Outline
1.  [Introduction](##introduction)
    1. [Code location](##code-location)
    2. [How to Deploy](##how-to-deploy)
    3. [Style Guide](##style-guide)
2.  [High level functionality](##high-level-functionality)
    1. [Code Structure](##code-structure)
    2. [Guiding Principles](##guiding-principals)
    3. [Operation](##operation)
3.  [SD Card](##sd-card)
    1. [Configuration](##sd-configuration)
    2. [Controller Parameters](##sd-controller-parameters)
3.  [Sensors](##sensors)
    1. [Structure](##sensor-structure)
    2. [Adding New](##adding-new-sensors)
4.  [Actuators](##actuators)
    1. [Structure](##actuator-structure)
    2. [Adding New](##adding-new-actuators)
    3. [T-motor Initialization](##t-motor-initialization) (Get info from Chance)
5.  [Controllers](##controllers)
    1. [Structure](##controller-structure)
    2. [Adding new](##adding-new-controllers)
    3. [Parameters](##controller-parameters)
6. [Bluetooth Communication](##bluetooth) (Get info from Chance)
7. [Resources](##resources)

***
## Introduction 
Hello, welcome to the guide.  
This guide is designed to provide background information on the new (at time of writing) code used to control the NAU Biomechatronics Lab's exo.
This system is designed to be flexible, where the system can be easily adapted to the user's needs, changing the motors used, number of joints, etc. 
 
### Code Location  
If you are reading this you have found the location, but for completeness it can be found at: https://github.com/naubiomech/ExoCode/tree/nano_teensy_board.
To access it you will need to get permission from one of the existing members.  
Most everyone is an owner and should be able to give you access.
This code is currently under the nano_teensy_board branch. 
Detailed instructions on setting up git can be found [here](https://docs.google.com/document/d/15j4S6pc9qAXF1_4HghfnPCgwyrsP_jZ15kv34wk7MbU/edit?usp=sharing).

### Style Guide  
The detailed style guide can be found [here](StyleGuide.md).

### How to Deploy 
The system consists of several components:
1. Actuators
2. Sensors 
3. Control Board
4. SD Card

First, you will need to connect the physical components.
1. Mount the motors on the system as appropriate.  
2. Connect the power and communication cables to the control board.
    - The connectors should attach to the side they are on while worn, e.g. the left motor connects to the left side of the board.
    - The connections top to bottom should be the proximal to distal joints used, e.g. if the hip and ankle are used the hip should be the top most location, the ankle should be next; if just the ankle is used it should be on the top most connector of the appropriate type.
3. Similarly, sensors should be connected on the side used, and if associated with a motor next to that motor, e.g. if the right ankle has a torque sensor it should go below the right ankle communication connection, regardless of if another joint is used. 
4. The control board may have multiple microcontrollers on it they should all be flashed with ExoCode.ino through the Arduino IDE.  The compiler will select the correct parts of the code to use if you select the correct microcontroller.    
    - Update /ExoCode/src/Config.h BOARD_VERSION with the version number found on the control board before compiling. 
    - [Arduino Instructions](https://docs.google.com/document/d/1ToLq6Zqv58Q4YEmf4SzqJDKCTp52LUaKgYg5vNmHdaI/edit?usp=sharing)
5. Lastly, is the SD card.
    - Transfer the content of the SD Card folder to the micro SD card. 
    - Update the config.ini file
        - Change the board version
        - Change the Exo name 
        - Go to the section for that name and confirm the settings match your system.
    - For the joints you are using, go to that folder and update the controllers you plan to use.

Those are the rough points.
Detailed explanations can be found in the coming sections.
    


***
## High Level Functionality
The system is broken into separate components that can be put together based on the system's needs.  
Arduino is used to control these components.   
There are two key classes, ExoData and Exo.  
ExoData is used to store the data recorded by the system and the data used to control the system and should mirror the structure of Exo.
The hierarchy is:
- Exo/ExoData 
    - StatusLed
    - SyncLed
    - FSRs
    - Leg/LegData
        - Joint/JointData
            - TorqueSensor
            - Motor/MotorData
            - Controller/ControllerData
            
More info can be found below in [Code Structure](#code-structure).

The high level way the code runs is:
1. Read the configuration from the SD card
2. Create the exo_data object (static in main loop)
3. Create the exo object (static in main loop)
4. Read new messages and update exo_data
5. exo.run() which runs the subcomponents as well 
        

### Code Structure 

[Namespaces](Namespaces.md) are used in place of global variables.  
They are used for items that need to be accessible by other parts of the code. 
They are used as little as possible to minimize the amount of dependencies as that makes the code less modular.

[Data Structure](ExoDataStructure.md) 

[Exo Structure](ExoStructure.md)

### Guiding Principals 

### Operation  

***
## SD Card  

### SD Configuration  

### SD Controller Parameters 

*** 
## Sensors

### Sensor Structure 

### Adding New Sensors

*** 
## Actuators

### Actuator Structure 

### Adding New Actuators 

### T-motor Initialization 

*** 
## Controllers 

### Controller Structure 

### Adding New Controllers

### Controller Parameters  

***
## Bluetooth 

***
## Resources 