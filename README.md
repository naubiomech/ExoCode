# Nano Teensy Board

This code is for the CAN based motors on boards where the nano handles low frequency sampling (eg. battery voltage), and Bluetooth communication; and the teensy handles high frequency sampling and controls.  
The two boards communicate through SPI/UART (still figuring it out).
Check out the [Documentation Folder](/Documentation) for details.


## Common Code (Mixed [in progress])
- [x] Configuration reading (Paul)
- [x] Data structure (Paul)
- [x] Modify Run methods to check Exo_Data and perform Exo tasks (Paul)
- [ ] Add a check to the controller to ensure that the user updates the controller parameters when changing 
controllers.


## Nano Specific Code (Chance [in progress])
### Classes
Chance I am going to let you handle most of this but I am going to provide some structure, feel free to change.  You can find the pins in the TMotor_Exo_PCB, WithTeensy branch.
- [X] Bluetooth handler
- [ ] SPI/UART Handler (Teensy Communication)
- [X] i2c handler (Battery Voltage)
- [ ] Use the modified torque command to set the controller from the sd card

## Teensy Specific Code (Mixed [in progress])
### Classes
We can tag which one of us is working on what when we get there.
- [X] Sync LED (Paul)
- [X] Status LED (Paul)
- [ ] SPI/UART Handler
- [X] Exo (Paul)
- [X] Leg (Paul)
- [X] Joint (Paul)
- [X] FSR (Paul)
- [X] Controller (Paul)
- [X] Motor (Chance [working on CAN]) I am giving you this one as you have more experience with CAN
- [X] TorqueSensor (Paul)

![Diagram](/Documentation/Figures/CodeDiagram.svg)

## Libraries
The libraries should be moved to C:\Users\\\[USER\]\Documents\Arduino\libraries\ or system equivalent.
Details on the libraries can be found in the [Libraries Folder](/Libraries).

## Optimizations
- SPI direct memory access. Should change SPI time from 20 &mu;s per byte to the time to write the memory.
    - Teensy Datasheet 48.2.1 G:\Shared drives\Biomech_Lab\Manuals_Guides_Forms\Microcontrollers\Teensy_4_1
        - Configure SPI
        
- BLEparser change from char representation to num bytes, expect 3 to 6x speed up.
- CAN direct memory access, should cut about 250 &mu;s per motor.
- CAN bus per leg after DMA, roughly cut CAN time in half. Main control may be limiting time at this point at about 600&mu;s. Write at top, do controls, read at bottom.

## MORE DETAILS TO COME
Probably need to create a consistent/shared SPI interface
