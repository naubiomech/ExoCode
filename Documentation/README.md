# New Code User Guide

## Outline
1.  [Background](#background)
1.  [Introduction](#introduction)
    1. [Code location](#code-location)
    2. [How to Deploy](#how-to-deploy)
    3. [Style Guide](#style-guide)
2.  [High level functionality](#high-level-functionality)
    1. [Code Structure](#code-structure)
    2. [Guiding Principles](#guiding-principals)
    3. [Operation](#operation)
3.  [SD Card](#sd-card)
    1. [SD Configuration](#sd-configuration)
    2. [SD Controller Parameters](#sd-controller-parameters)
3.  [Sensors](#sensors)
    1. [Sensor Structure](#sensor-structure)
    2. [Adding New Sensors](#adding-new-sensors)
4.  [Actuators](#actuators)
    1. [Actuator Structure](#actuator-structure)
    2. [Adding New Actuators](#adding-new-actuators)
    3. [T-motor Initialization](#t-motor-initialization) (Get info from Chance)
5.  [Controllers](#controllers)
    1. [Controller Structure](#controller-structure)
    2. [Adding New Controllers](#adding-new-controllers)
    3. [Controller Parameters](#controller-parameters)
6. [Bluetooth Communication](#bluetooth) (Get info from Chance)
    1. [Bluetooth Background](#bluetooth-background)
    2. [Bluetooth Structure](#bluetooth-structure)
    3. [Sending a Message](#sending-a-message)
7. [Debugging](#debug) 
8. [Resources](#resources)
    1. [Lab Resources](#lab-resources)
    1. [C++ Resources](#c-resources)
    1. [Bluetooth Resources](#bluetooth-resources)
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

#### Arrays

#### Void

#### Modifiers

##### Signed vs Unsigned

##### Long and Short

##### Static

##### Volatile

##### Extern

### Functions

### Classes

#### Inheritance

#### Friend Classes

#### Abstact Classes

#### Initializer List

### Pointers

#### Function Pointers

### .h vs .cpp files
You may notice two files with the same name but different extensions, one .h and one .cpp.
The .h file is known as a [header file](https://www.learncpp.com/cpp-tutorial/header-files/) and is a place where you declare different items you want to use in a different file.
If you look at the top of the Arduino code [ExoCode.ino](/ExoCode/ExoCode.ino) you will see includes like:
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
```
void ExoData::reconfigure(uint8_t* config_to_send) 
{
    left_leg.reconfigure(config_to_send);
    right_leg.reconfigure(config_to_send);
};
```
So when we call reconfigure for the ExoData objects we call the reconfigure member functions for the left_leg and right_leg objects the class contains.


***
## Introduction   
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
    - Update the libraries. Move the files/folders in the [Libraries Folder](/Libraries). To your local Folder C:\Users\[USER]\Documents\Arduino\libraries\ or system equivalent.  Details on the libraries that are used are used can be found in the main [README](/README.md#libraries)
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
A subset of the firmware can run on a separate microcontroller, intended to handle Bluetooth communication and soft real-time functionality. The main microcontroller 
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

[Namespaces](Structure/Namespaces.md) are used in place of global variables.  
They are used for items that need to be accessible by other parts of the code. 
They are used as little as possible to minimize the amount of dependencies as that makes the code less modular.

[Data Structure](Structure/ExoDataStructure.md) 

[Exo Structure](Structure/ExoStructure.md)

### Guiding Principals 
The guiding principals of the code is to make it adaptable and modular. 
There are still some shortcomings with how we achieved this due to the nature of the dual microcontroller system but overall it should work well.
To this end we have utilized abstract classes for things like the motors where we define an interface so if we need to add motors that work in a different way, e.g. CAN vs PWM, we don't have to change the rest of the code just add the underlying private member functions.
Additionally sensors do not need access to the ExoData object, we considered doing this for all IO but decided it didn't make sense in all cases.

### Operation  
TODO: Update when the app portion is incorporated.

***
## SD Card  
The files for the SD card can be found in the [SDCard](/SDCard/) folder in the main directory.
The contents of this file should be copied to the root of the SD card, e.g. when you open the SD Card you should see config.ini.
The file contains the configuration file and the parameter files for the controllers.
These parameter files are a temporary measure till the new app is running.

### SD Configuration  
[config.ini](/SDCard/config.ini) is used to tell the code how they system is configured.
The fields should all be less than 25 characters as that is limited by the size of the array that is parsing it. 
The file is broken into sections denoted by being in \[ \], e.g. \[Board\], containing information related to the board.  
This separates information that is related into groups.
Within the section you have keys, these contain the information, e.g. version = 0.1.
The key names shouldn't be modified, version, as the parser is looking for the specific name but the value can be, 0.1.

We have some premade exoskeleton configurations you can choose from by putting their name in the Exo section.
Just check to make sure the settings in that section match your system.
If we are using a bilateral hip system we would set ```[Exo] name = bilateralHip```, then go to the section \[bilateralHip\] and check it matches the system we are using.
- sides - are you using the left, right, or both sides.
- hip, knee, ankle - sets the type of motor the joint uses.
- gear ratio - sets the transmission ratio for the joint torque to the motor output torque.  If the motor has a built in gearbox that should not appear here.
- default controller - is the controller the system starts with.
- flip dir - is if the direction of the motor values should be flipped.  For example if we have two motors pointing in towards the hip and both rotate counter clockwise with a positive current one of them will need to be sent a negative current so they both rotate in the same direction on the body.  When a side is flipped the commands and angle measurements will be inverted automatically so sending a positive command to both motors will have them move the body in the same way.

### SD Controller Parameters 
The parameters for each controller are stored in their corresponding joint folder.
This way if both joints are using a zero torque controller but need different gains they can pull from different files.
The files are comma separated value files, so there are commas between cells.
The first cell in the *first* row contains the number of lines in the header, how many lines we need to get through to get to the parameters.
The first cell in the *second* row contains the number of parameters to read.
The rest of the header just contains useful info for the person, such as the parameter order.
The first parameter row will be the default values, set 0.
The nth parameter row is n-1 parameter set, e.g. parameter row 2 will be referenced as set 1.

The order of the parameters should match how they appear in the parameter array which can be found in [ControllerData.h](/ExoCode/src/ControllerData.h). in the controller_defs namespace.

These will be selected using  the update torque field in the app where you set the joint, controller, and parameter set.

*** 
## Sensors
Sensors do not have a shared interface(abstract class), although you could do this if you want.
The sensors are designed to be stand alone so they do not need something like access to an ExoData object.
With this they must be written so that they take in the information they need and return the info they need.

For example the for the FSR to calibrate over a period of time they need to take in a command to calibrate but also to return when the calibration is finished.

### Sensor Structure 
The main thing the sensors will need is a constructor to setup the interface.
For most of the sensors they are just analog sensors so they will need the analog pin that is used.
For some sensors though you may need to define a communication protocol like SPI or I<sup>2</sup>C.
With these other interfaces you will need to make sure not to create conflicts with other systems using that interface.

### Adding New Sensors
Details can be found in [Adding New Sensors](AddingNew/AddingNewSensors.md)


*** 
## Actuators
Actuators are setup so that the system can add multiple types of motors and select the correct one for the system at startup.
The Joint instance will use a pointer to a motor instance.
This motor instance will be set based on what is in the config.ini file on the SD card.
To be able to call any type of motor we need to have a common interface which will be described next.

### Actuator Structure 
As with most of the system there is a parallel data structure that follows the system structure.
MotorData details can be found in [Data Structure](Structure/ExoDataStructure.md), but contains state and configuration information.  

The motors should all inherit their interface from the abstract class _Motor  in [Motor.h](/ExoCode/src/Motor.h).
This defines how other systems can call motors, that way the rest of the system doesn't need to know what specific motor you are using as they all have the same calls.
Within this you can then define what that call does for the specific motor/type.
With the CAN motors they have a separate class that this type of motor inherits since they all work in much the same way but have some parameters that are different.
You can see this in the Motor.h file as 
```
class _CANMotor : public _Motor
```
and
```
class AK60 : public _CANMotor
```

Where _CANMotor inherits from _Motor and then the AK60 motor inherits from the _CANMotor class so it also gets the things that are in _Motor.
More info on inheritance can be found on [tutorialspoint](https://www.tutorialspoint.com/cplusplus/cpp_inheritance.htm) or [w3schools](https://www.w3schools.com/cpp/cpp_inheritance.asp).

We decided that the motors would always be used in torque control mode so transaction(torque) and send_data(torque), only take torque commands.
If you need a position/velocity controller you will need to make this as a separate controller.
This was done since most any motor will have access to torque controller, even if it is just driving current, but may not have other more advanced built in controllers.


### Adding New Actuators 
Details can be found in [Adding New CAN Motor](AddingNew/AddingNewCanMotor.md).
This is specifically for the TMotor CAN motors but can be adapted to new types of motors when we have them.

### T-motor Initialization 
TMotor initialization information can be found in: [G:\Shared drives\Biomech_Lab\Manuals_Guides_Forms\TMotor AK Resources\Manuals](https://drive.google.com/drive/folders/1Zrfk-qxY8917pJ-qeVlzmoCAqcmfqqJG?usp=sharing)

*** 
## Controllers 


### Controller Structure 


### Adding New Controllers
Details can be found in [Adding New Controller](AddingNew/AddingNewController.md).

### Controller Parameters 
 
 
#### Hip
- [Stasis](Controllers/Stasis.md)
- [Zero Torque](Controllers/ZeroTorque.md)
- [Sine](Controllers/Sine.md)
- [User Defined](Controllers/UserDefined.md)
- [Heel Toe](Controllers/UserDefined.md)
- [Extension Angle](Controllers/Hip/ExtensionAngle.md)
- [Bang Bang](Controllers/Hip/BangBang.md)
- [Franks Collins Hip](Controllers/Hip/FranksCollinsHip.md)


#### Knee
- [Stasis](Controllers/Stasis.md)
- [Zero Torque](Controllers/ZeroTorque.md)
- [Sine](Controllers/Sine.md)
- [User Defined](Controllers/UserDefined.md)

#### Ankle
- [Stasis](Controllers/Stasis.md)
- [Zero Torque](Controllers/ZeroTorque.md)
- [Sine](Controllers/Sine.md)
- [User Defined](Controllers/UserDefined.md)
- [Proportional Joint Moment](Controllers/Ankle/ProportionalJointMoment.md)
- [Zhang Collins](Controllers/Ankle/ZhangCollins.md)

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
```
BleMessage batt_msg = BleMessage();
batt_msg.command = ble_names::send_batt;
batt_msg.expecting = ble_command_helpers::get_length_for_command(batt_msg.command);
batt_msg.data[0] = _data->battery_value;
_exo_ble->send_message(batt_msg);
```

***
## Debug
In the top of many of the files you will see a define for debugging like ```#define EXO_DEBUG 1```.
When this is present debug statements will print if they are in an ```#ifdef``` like:
```
#ifdef EXO_DEBUG
    Serial.println("Exo :: Constructor : _data set");
#endif
```
This is because serial printing is a pretty slow process, so you only want to do it if you are actively using it.
So if you are adding a print statement you should wrap it in an ```#ifdef``` for that file.

The reason we do it file by file rather than printing everything is because it allows you to focus in on the area you are working on.
Even within this you may still want to comment out some of the prints within the file to really focus on the area you are using.

***
## Resources
### Lab Resources
[Arduino Instructions](https://docs.google.com/document/d/1ToLq6Zqv58Q4YEmf4SzqJDKCTp52LUaKgYg5vNmHdaI/edit?usp=sharing)

### C++ Resources
- [tutorialspoint](https://www.tutorialspoint.com/cplusplus/index.htm) 
- [w3schools](https://www.w3schools.com/cpp/default.asp)

### Bluetooth Resources