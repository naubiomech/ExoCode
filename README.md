# Nano Teensy Board

This code is for the CAN based motors on boards where the nano handles low frequency sampling (eg. battery voltage), and Bluetooth communication; and the teensy handles high frequeny sampling and controls.  The two boards communicate through SPI.

## Common Code (Mixed [in progress])
- [x] Configuration reading (Paul)
- [x] Data structure (Paul [Structure done, may need to add as we go])

## Nano Specific Code (Chance [in progress])
### Classes
Chance I am going to let you handle most of this but I am going to provide some structure, feel free to change.  You can find the pins in the TMotor_Exo_PCB, WithTeensy branch.
- [ ] Bluetooth handler
- [ ] spi handler (Teensy Communication)
- [ ] i2c handler (Battery Voltage)

## Teensy Specific Code (Mixed [in progress])
### Classes
We can tag which one of us is working on what when we get there.
- [X] Sync LED (Paul [Make style compliant])
- [X] Status LED (Paul [Make style compliant])
- [ ] SPI Handler (Chance [not started]) I added this to Chance since it seems tied to the Nano SPI, so it will be easier for an individual to debug.
- [X] Exo (Paul [structure in place need to add function])
- [X] Leg (Paul [structure in place need to add function])
- [X] Joint (Paul [structure in place need to add function])
- [X] FSR (Paul [structure in place need to add function])
- [X] Controller (Paul [structure in place need to add function])
- [ ] Motor (Chance [working on CAN]) I am giving you this one as you have more experience with CAN
- [X] TorqueSensor (Paul [structure in place need to add function])

![Diagram](CodeDiagram.svg)

MORE DETAILS TO COME
Probably need to create a consistent/shared SPI interface
