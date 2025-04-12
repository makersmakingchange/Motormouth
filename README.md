# Motormouth

## Introduction 
Hackathon 2019 project partnered with the Neil Squire Society to use the LipSync mouth controlled joystick to control an RC car.

## Getting Started
1.  Prerequisites:
    * Arduino UNO
    * Arduino IDE (https://www.arduino.cc/en/Main/software)
2.  Install the MCP4261 library
    * Launch the Arduino IDE
    * Sketch > Include Library > Add .ZIP Library ...
    * Select the MCP4261 subfolder in this repo and press "Open"

## Building
1.  Build and deploy the modified LipSync Arduino sketch
    * Connect LipSync to PC via USB
    * Enter the Software sub-directory
    * Open LipSync_Motormouth_Firmware/LipSync_Motormouth_Firmware.ino in Arduino IDE
    * Configure COM port if necessary
    * Upload sketch to LipSync
2.  Build and deploy the Motormouth Arduino sketch
    * Connect Motormouth MCU to PC via USB
    * Open MOTORMOUTH_RX/MOTORMOUTH_RX.ino in Arduino Include
    * Configure COM port if necessary
    * Upload sketch to Motormouth MCU

## Pairing LipSync with Motormouth
	* For this build the pairing of the Lipsync and Rx Hardware was done manually. I found the mac address of the Lipsync by using BluetoothView on PC and then setting auto connect on the Rx Bluetooth Mate manually.

## Testing
1.  Turn on LipSync
2.  Turn on Motormouth
3.  The LED on Motormouth's Bluetooth module should turn green once LipSync and Motormouth are paired
4.  Turn on RC controller
5.  Turn on RC car
6.  Testing:
    * Move LipSync joystick left and right to control RC car steering
    * Puff gently to increase forward speed (or decrease reverse speed).
    * Sip gently to increase reverse speed (or decrease forward speed).
    * Puff hard to go "full throttle"
    * Move LipSync joystick down to stop RC car
	
## Hardware Setup
The schematics for the project can be found on <a href="https://upverter.com/eda/embed/#designId=63d9d49c4f5af5d4">Upverter</a>.



## Known Issues
*   Sometimes, the Motormouth module fails to connect to the LipSync joystick.  Other times, the link between the two seems to be broken and commands stopped being received.  These problems may be resolved by power cycling Motormouth's Bluetooth module.  The root causes of these issues are not known.

<!-- ABOUT MMC START -->
## About Makers Making Change
[<img src="https://raw.githubusercontent.com/makersmakingchange/makersmakingchange/main/img/mmc_logo.svg" width="500" alt="Makers Making Change Logo">](https://www.makersmakingchange.com/)

Makers Making Change is a program of [Neil Squire](https://www.neilsquire.ca/), a Canadian non-profit that uses technology, knowledge, and passion to empower people with disabilities.

Makers Making Change leverages the capacity of community based Makers, Disability Professionals and Volunteers to develop and deliver affordable Open Source Assistive Technologies.

 - Website: [www.MakersMakingChange.com](https://www.makersmakingchange.com/)
 - GitHub: [makersmakingchange](https://github.com/makersmakingchange)
 - Bluesky: [@makersmakingchange.bsky.social](https://bsky.app/profile/makersmakingchange.bsky.social)
 - Instagram: [@makersmakingchange](https://www.instagram.com/makersmakingchange)
 - Facebook: [makersmakechange](https://www.facebook.com/makersmakechange)
 - LinkedIn: [Neil Squire Society](https://www.linkedin.com/company/neil-squire-society/)
 - Thingiverse: [makersmakingchange](https://www.thingiverse.com/makersmakingchange/about)
 - Printables: [MakersMakingChange](https://www.printables.com/@MakersMakingChange)

### Contact Us
For technical questions, to get involved, or to share your experience we encourage you to [visit our website](https://www.makersmakingchange.com/) or [contact us](https://www.makersmakingchange.com/s/contact).
<!-- ABOUT MMC END -->
