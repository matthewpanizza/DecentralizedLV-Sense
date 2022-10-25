# DecentralizedLV-Sense
A repository for the SolarPack Decentralized Low Voltage Switch Sensor CAN Boards. The sense board is connected to switches near the steering wheel (such as turn signals, headlights, brake lights) and sends messages to the LPDRV boards over CAN to control lights, fans, pumps, and other elements.

## Hardware Capabilities

- CANBus control with 250K, 500K, and 1Mbps speeds
- 10 Voltage-divided inputs
- Analog capability on some pins for measuring system voltages
- 2 digital and DAC outputs for convenience
- 12V supply breakout

## Software Capabilities

- CANBus transmit and receive for reading data from units that provide temperature/power data
- 10 millisecond response time
- Easy functions to read all pins and a function to parse out states based on the pin statuses
- Temperature-based control of devices based on data received by computer

## PCB Design

![IMG_2090](https://user-images.githubusercontent.com/47908040/197679575-06090c74-d04c-44bf-b865-fdd938136e2a.jpg)
