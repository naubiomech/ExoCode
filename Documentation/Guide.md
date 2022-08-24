# New Code User Guide

## Outline
1.  [Background](##background)
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
    1. [Background](##bluetooth-background)
    2. [Structure](##bluetooth-structure)
    3. [Sending a Message](##sending-a-message)
7. [Resources](##resources)

***
## Background 
The code is written Arduino and C++.
This background section is intended to give you and understanding about the different things you may see in the code and how to interpret them.
More detailed information can be found on the internet, I like [tutorialspoint](https://www.tutorialspoint.com/cplusplus/index.htm) and [w3schools](https://www.w3schools.com/cpp/default.asp) but there are other resources that are good.

### Bits and Bytes

### Addresses

### Variables

### Types

#### Bool

#### Char

#### Int

#### Float

#### Double

#### Void

#### Modifiers

##### Signed vs Unsigned

##### Long and Short

##### Static

##### Volatile

### Functions

### Classes

#### Initializer List

### Pointers

#### Function Pointers

### .h vs .cpp files
You may notice two files with the same name but different extensions, one .h and one .cpp.
The .h file is known as a [header file](https://www.learncpp.com/cpp-tutorial/header-files/) and is a place where you declare different items you want to use in a different file.
If you look at the top of the Arduino code [ExoCode.ino](https://github.com/naubiomech/ExoCode/tree/nano_teensy_board/ExoCode) you will see includes like:
```
#include "src\ExoData.h"
```
This tells the code that you want to use the stuff that is declared in that file, this example file declares a class called ExoData that we use to store data.

I have said declare a bunch of times now but it may be unclear what that means.
A "declaration" tells the compiler that puts everything together for the processor what things are available and how they are called, but nothing about what they do.
In our example we have a class named ExoData which contains some other stuff, like the classes, functions, and variables we already discussed.
Within this class there is a member function ```void reconfigure(uint8_t* config_to_send); ```, so the complier knows that we can call reconfigure if we give it a uint8_t pointer and it won't send anything back.
What happens when we call it?
The compiler doesn't care at this point, it just wants to know that we can use it.
Similarly there are some variables inside that we can also call, ```bool estop;``` is a Boolean that lets us know the status of the emergency stop button, but we can also store objects for other classes like ```LegData left_leg;```.
```
class ExoData 
{
	public:
        ExoData(uint8_t* config_to_send); // constructor
        void reconfigure(uint8_t* config_to_send);
        void for_each_joint(for_each_joint_function_t function);
        
        uint16_t status;
        bool sync_led_state;
        bool estop;
        float battery_value; // Could be Voltage or SOC, depending on the battery type
        LegData left_leg;
        LegData right_leg;
};
```

So when we want to actually say what values the variables have or what happens when we call the function we need to "define" them.
This is where the .cpp file comes in.
If we want to define what happens when we call reconfigure for an ExoData object we code it out 
```void ExoData::reconfigure(uint8_t* config_to_send) 
{
    left_leg.reconfigure(config_to_send);
    right_leg.reconfigure(config_to_send);
};
```
So when we call reconfigure for the ExoData objects we call the reconfigure member functions for the left_leg and right_leg objects the class contains.


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
A subset of the firmware can run on a seperate microcontroller, intended to handle bluetooth communication and soft real-time functionality. The main microcontroller 
communicates with the communication microcontroller via SPI. 
The hierarchy is:
- ComsMCU/ExoData
    - ExoBLE/ExoData
        - BleParser
            
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
The system uses Bluetooth Low Energy (BLE) to communicate with a graphical user interface (GUI). For an introduction to BLE, [see](https://learn.adafruit.com/introduction-to-bluetooth-low-energy).

### Bluetooth Background
The Exosekeleton uses Norduc's UART Service (NUS) to communicate with the GUI. This service has RX and TX characteristics mimicking UART. In order for the app to connect with the Exoskeleton it's name must beging with "EXOBLE_" and advertise the NUS. When the Exoskeleton connects with the GUI, it will begin sending battery data. When a trial is started the device will begin transmitting a subset of the ExoData struct. 

### Bluetooth Structure
The CommsMCU class is the highest class in the Communications firmware heirarchy. It contains the battery object, and the ExoBLE object. This class manages the bluetooth connection and data. The class also performs battery sampling. 
The ExoBLE class handles all bluetooth work. This includes initialization, advertising, connection, and data transfer. 
The BleParser class is used to serialize and deserialize the BLE data. The application uses Nordic's UART service to pass all of the data. Therefore, the command-data pairs must be packaged together and unpackaged on the peripheral and central.
There are several variables in config.h that control the timing of data transmission. 

### Sending a Message
If you would like to add a new message, see the "AddingNewBLEMessage" doc. The messages are all created in the ble_commands.h file. ble_commands.h also defines the functions that are called when a command is received. To send a new message you must package a BleMessage object with your desired command and data. The data must be packaged correctly, both in length and index. Currently there is no method to ensure the correct index is used for a specific command, but the length of the commands can be found in the ble namespace. Here is an example message (Sending messages must be done in the ComsMCU):
    BleMessage batt_msg = BleMessage();
    batt_msg.command = ble_names::send_batt;
    batt_msg.expecting = ble_command_helpers::get_length_for_command(batt_msg.command);
    batt_msg.data[0] = _data->battery_value;
    _exo_ble->send_message(batt_msg);


***
## Resources 