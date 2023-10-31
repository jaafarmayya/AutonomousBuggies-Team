
# WRO2023 FUTRE ENGINEERS - Autonomous Robot Vehicle

This repository presents a detailed description about the autonomous robot vehicle used by team 'Autonomous Buggies' in the aim of participating in WRO2023 competetion - Future Engineers category. 
The mechanical, electrical, and electronic parts of the vehicle are well described and provided. The sourcec codes used to program the vehicle are written in Python and Arduino C. Additionally, a detailed explanation of the algorithms and mathematical principles is provided to help everyone learn. All these details can be found the sections listed below:

---

##Introduction

This Repository contains all necessary information to entirely build a self driving car, which is capable of completing three laps and
handling different configurations of the obstacles. The code is divided into two main parts, one written with python programming language, which is responsible for image processing and sending made decision to the arduino via Serial communication ([Python code](https://github.com/jaafarmayya/AutonomousBuggies-Team/blob/main/src/Obstacle%20Challenge/Computer%20Vision%20-%20Serial%20Connection.py)). And one written with Arduino C programming language, which is responsible for controlling the motors movement based on the different sensors reading and recieved decision ([Arduino code](https://github.com/jaafarmayya/AutonomousBuggies-Team/tree/main/src)). The rest of the readme.md file explains in deteails each of the following:

- [Software Setup](#SoftwareSetup)
- [Mobility Management](#MobilityManagement)
- [Obstacle Management](#ObstacleManagement)
- [Power and Sense Management](#PowerandSenseManagement)

---

## Software Setup <a id="SoftwareSetup"></a>

Before using the robot, software should be configured first, you will need to install the following (Programs have been tested and configured with both Windows 10 and Windows 11 operating systems. In case of using another operating system, we strongly recommend using any available searching engine to search for the installation for the software by yourself).
We'll be using Raspberry Pi to run image processing code written in python programming language. Commands will be sent via Serial communication to arduino wich will control the motors.

###Raspberry Pi Preparation and Configuration
In this section, we will go through all the steps to prepare and configure your Raspberry Pi. After completing the steps, you will be able to access your Raspberry Pi remotely and control it.
####Raspberry Pi Operating System 
There are a lot of OS systems for Raspberry Pi with different versions. You can choose any of them from [official site](https://www.raspberrypi.com/software/operating-systems/), but it is preferable to install the same version we installed since it is tested and it works fine ([download link](https://downloads.raspberrypi.com/raspios_full_arm64/images/raspios_full_arm64-2023-02-22/2023-02-21-raspios-bullseye-arm64-full.img.xz)).
####Raspberry Pi Imager installation
You will be setting up Raspberry Pi OS on your Raspberry Pi using the Raspberry Pi Imager tool, it will allow you to flash the image onto a USB flash drive. The provided steps cover the installation process, initial configuration, and essential steps to get your Raspberry Pi up and running. 
Using the Raspberry Pi Imager tool to flash the Raspberry Pi OS image onto a USB flash drive offers a convenient and efficient way to set up your Raspberry Pi. The Imager simplifies the installation process by providing a user-friendly interface to select the operating system and target storage medium. With just a few clicks, you can download the official Raspberry Pi OS image and write it directly to the USB flash drive. This eliminates the need for manual installation and configuration, saving you time and effort.
- To install Raspberry Pi Imager, Visit the official Raspberry Pi website at https://www.raspberrypi.org/software/.
- Download the Raspberry Pi Imager tool suitable for your operating system (Windows, macOS, or Linux).
- Install Raspberry Pi Imager on your computer by following the provided instructions.
####Flashing the Raspberry Pi OS Image to USB
After downloading Raspberry Pi OS and installing the Imager, you can flash the OS Image to your USB (SD card can also be used):
 - Launch Raspberry Pi Imager on your machine.
 - In the Raspberry Pi Imager window, click on "Choose OS" button.
 - Click on "Use custom" option, then navigate to the directory of the OS that was previously downloaded.
 - Click on "Choose  Storage" option, in the file explorer window select your USB flash drive as the target storage medium.
 - The widnow should have the same configuration as the image below after choosing the flash drive and OS:
     
      ![MobaXterm](/Documentation/Images/RaspberryPiOS/1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 1: RaspberryPi Imager </i></p>

 - Now click on the settings icon in the bottom right corner, you will need to enable the SSH option and choose a username and a password (This is necessary because you will use SSH to establish a connection between your device and Raspberry Pi, it will allow you to control the Raspberry Pi remotely).

  ![MobaXterm](/Documentation/Images/RaspberryPiOS/2.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 2: RaspberryPi SSH connection enabling </i></p>
  ![MobaXterm](/Documentation/Images/RaspberryPiOS/3.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 3: RaspberryPi username and password </i></p>

- Next, you will need to enter the name of your network and its password, this will allow you to access the Raspberry pi directly and remotely.
   
   ![MobaXterm](/Documentation/Images/RaspberryPiOS/4.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 4: RaspberryPi network configuration </i></p>

- Click on the "Write" button to start the flashing process. Raspberry Pi Imager will download the latest Raspberry Pi OS image and write it to the USB flash drive. This process may take several minutes, depending on your machine.
Once the flashing process is completed, Raspberry Pi Imager will display a success message.

### Raspberry Pi Remote Connection configuration
There three ways to control Raspberry Pi:
- Use a mouse, a monitor and a keyboard attached to your Raspberry Pi.
- Use ethernet cable attached to your machine and Raspberry Pi, and a virutal machine software.
- Connect both Raspberry Pi and your machine to the same network, and use a virtual machine software.

we will be using the third option since it is be a lot easier to control your Raspberry Pi remotely.
1. To control and access the Raspberry Pi, you will need a program to establish wireless connection. MobaXterm is the program that will allow you to establish this communication, which is based on SSH protocol.
 - Download the program from this [link]( https://mobaxterm.mobatek.net/download-home-edition.html), choose portable edition.
 - After succefully downloading the software, extract the contents of the downloaded file to a specified folder.
 - The program is ready to be used now, when launching it, a window like that will appear (it should be noted that both the raspberry pi and your device have to be connected to the same network):

   ![MobaXterm](/Documentation/Images/MobaXterm/1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 5: MobaXterm Interface </i></p>

 - Choose Session tab in the top left corner:

   ![Session Option](/Documentation/Images/MobaXterm/2.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 6: Choosing new session </i></p>

 - A new window will appear that contains the settings of the new connection:

   ![Session window](/Documentation/Images/MobaXterm/3.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 7: new session tab </i></p>

 - Click on the SSH icon, we will be using SSH in our work. SSH (Secure Shell) is a network protocol that provides a secure way to access and manage remote systems. It allows to establish a secure encrypted connection between local computer and a remote device, such as a Raspberry Pi in our case.Using SSH to access a Raspberry Pi offers several advantages, as it allows remote access to the Raspberry Pi's command-line interface, providing a convenient and efficient way to manage and configure the device. Through an SSH connection, commands can be executed, files can be transfered, perform system administration tasks without the need for physical access to the Raspberry Pi. Moreover, SSH is particularly useful for headless Raspberry Pi setups, where the device is not connected to a monitor or keyboard. In such cases, SSH enables remote administration, making it possible to configure and control the Raspberry Pi over the network from a different computer.

   ![SSH](/Documentation/Images/MobaXterm/4.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 8: choosing SSH communication  </i></p>

- To establish a connection with the Raspberry Pi, remte host name will be needed, which will be raspberrypi.local (The IP of the Raspberry Pi can also be used, but the domain name raspberrypi.local will work fine). With current Configuration, all commands will be executed through the terminal, to make these operations easier, LXDE desktop will be used as GUI as in the image below:

   ![Settings Configuration](/Documentation/Images/MobaXterm/6.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 9: Remote Host and LXDE Desktop</i></p>

- Now the settings are ready, click ok and it will need the username and password that was set during the setup of the raspberrypi py operating system:

   ![Settings Configuration](/Documentation/Images/MobaXterm/7.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 10:Login window </i></p>

- After applying the previous steps, connection is succefully established and Raspberry Pi is ready to be used remotely, a new LXDE desktop window will be opened, all subsequent work will be done here:
  
   ![Settings Configuration](/Documentation/Images/MobaXterm/8.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 11: RaspberryPi LXDE desktop </i></p>

- Update Raspberry Pi, enter the following commands in order, this step make take some time:

   1- ```sudo apt update``` 

   2- ```sudo apt upgrade```

### Raspberry Pi Camera Activation

- One last step is to turn on Raspberry Pi camera. Write the command below in the terminal:

   ```sudo raspi-config```

- The window of Raspberry Pi settings will be opened, use arrow keys to move the cursor between the options. Choose the option with label "Interfacing Options" by pressing "Enter" key:

  ![MobaXterm](/Documentation/Images/RaspberryPiOS/cam1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 12: Interfacing Options </i></p>

- Now choose camera and press "Enter", the camera is activated now.
  
    ![MobaXterm](/Documentation/Images/RaspberryPiOS/cam2.png)

    ![MobaXterm](/Documentation/Images/RaspberryPiOS/cam3.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 13+14:Choosing camera option </i></p>

- After finishing the previous steps, press "ESC" to return to the original terminal, and enter the command below:
 
  ```sudo reboot```

### Installing Arduino IDE on Raspberry Pi

- Arduino IDE installation steps are straightforward. Open the terminal by clicking on the terminal icon or by using the shortcut Ctrl+Alt+T, then enter the following command:

   ```sudo apt install arduino```

- Before using the IDE, setting the permissions for the Arduino port in Raspberry Pi is needed:
Connect the Arduino to the Raspberry Pi using a USB cable. In the terminal on the Raspberry Pi, type the following command to list the available ports on your Raspberry Pi:
  
   ```ls /dev/tty*```

- This command will display a list of devices, and the Arduino port should be listed as something like `/dev/ttyACM0` or `/dev/ttyUSB0`. Note down the port name as you will need it for the next step. By default, the serial ports in Raspberry Pi are not accessible to regular users. To grant access to the Arduino port, you can use the chmod command to change the permissions of the port. Run the following command, replacing /dev/ttyACM0 with the actual port name you obtained:

   ```sudo chmod a+rw /dev/ttyACM0```

- After running the chmod command, you should now have the necessary permissions to access the Arduino port, and you are ready to use the arduino with the raspberry pi. To use the Arduino IDE, you can easily access it in the programming tab as in the image below:
   
   ![Settings Configuration](/Documentation/Images/MobaXterm/34.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 15:Open programming tab and click Arduino icon </i></p>

- A window will open like the one below:

   ![Settings Configuration](/Documentation/Images/MobaXterm/11.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 16:Ardunio IDE interface</i></p>

### Downloading OpenCV PySerial library

- PySerial library is essntial component to establish a succeful connection between Arduino and Raspberry Pi, it can be easily downloaded using the following command below:

   ```sudo apt-get install python-serial```

### Downloading OpenCV library on Raspberry Pi

We will be using OpenCV library, which stands for (Open Source Computer Vision Library). OpenCV is a popular open-source computer vision and machine learning library. It provides a vast collection of functions and algorithms for image and video processing, object detection and tracking, feature extraction, and more.

- OpenCv library isn't installed by default on the Raspberry Pi, you will have to install it. Open the terminal, and type the following command:

   ``` sudo apt-get install python3-opencv```
- Wait till the installation is done, when it is done, open any code editor, which can be found in the programming tab:
  
     ![Settings Configuration](/Documentation/Images/MobaXterm/34.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 17:Open programming tab and click Geany icon</i></p>

- Choose Geany code editor for example. Geany is a simple and easy to use code editor (Try searching the internet for more details in case you want to know more), a window like this will be opened:
  
     ![Settings Configuration](/Documentation/Images/MobaXterm/9.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 18:Geany editor interface </i></p>

- Import the library using the following command, and you are ready to use the library:

   ```import cv2```

---

##Mobility Management <a id="MobilityManagement"></a>

### Vehicle Main Body
The vehicle main body is built using `LEGO`, building instructions are available [here](https://github.com/jaafarmayya/AutonomousBuggies-Team/blob/main/models/Building%20Instructions.rar).
The main Micrprocessor in the vehicle is `Raspberry Pi 4 Model b` provided with `Raspberry pi Camera V2.1`.
Main Macrocontroller is Atmega mounted on `Arudino Mega 2560` board.
To ensure the accurate and efficient control and direction of the vehicle’s movement, we employed two crucial components:
- **Servo Motor Model**
- **J Sumo DC Motor**
- **BTS7960 Motor Driver**

The servo motor model was selected for controlling the vehicle’s steering, while the BTS7960 motor driver is utilized to control the movement of J Sumo DC motor in order to provide the necessary power to drive the vehicle’s motion. 

### The Motion Mechanism 
We used Direct-Acting steering and to achieve that a servo motor was installed in way that can move the steering mechanism freely, while the DC motor (J Sumo 1000 rpm) serves as the driving motor. The kinetic energy generated by the DC motor is transferred to the back wheels through a differential gearbox. The wheels receive this energy via a drive shafts of LEGO components, which transmits torque to the differential gearbox through a suitable coupler [model](https://github.com/jaafarmayya/AutonomousBuggies-Team/blob/main/models/Coupler.jpg).


###Steering Mechanism

Our design incorporates front Direct-Acting Steering, a mechanism that eliminates the need for a steering gear by connecting the steering wheel directly to the steering linkage. Our front direct-acting steering method offers several advantages over conventional steering systems:

**A.Enhanced Responsiveness:**
The direct connection between the steering wheel and the steering linkage results in immediate and precise response to servo's input (angle), providing enhanced control and maneuverability.

 **B.Reduced Complexity:**
Our steering method simplifies the overall design, reducing weight and complexity, and potentially lowering manufacturing and maintenance costs.

**C. Lower Maintenance Requirements:**
 With fewer components involved, the direct-acting steering method can reduce the need for maintenance and potential points of failure, resulting in improved reliability and durability.
The mechanism of our front direct-acting steering method can be summarized as follows:

**-  Steering Input Translation:**
 When the servo motor turns the steering axle, the input is directly transmitted to the steering linkage, causing the front wheels to turn accordingly.

**-  Cornering:**
 During cornering, the steering mechanism maintain a smooth trajectory based on the servo motor input.

![Direct Acting Steering](/Documentation/Images/Steering+Mechanism/Picture1.jpg)

![Direct Acting Steering](/Documentation/Images/Steering+Mechanism/SteeringMechanism.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 19:Direct Acting Steering </i></p>


When designing a steering mechanism, some parameters should be considered to measure the characteristics and specifications of the steering system:

**1. Steering Ratio**

It is a key parameter in a steering system that describes the relationship between the rotation of the steering wheel (which is servo axle in our case) and the resulting rotation of the wheels. Moreover, it quantifies the mechanical advantage or amplification provided by the steering mechanism.
Mathematically speaking, it is defined as the ratio of the angle turned by the steering wheel (servo axle) θ<sub>servo</sub> 
to the angle turned by the wheels θ<sub>wheels</sub> , in the direct acting steering θ<sub>wheels</sub> is equal to θ<sub>servo</sub>.
It can be expressed as:

$\theta _{servo} =  \theta _{wheels}  \Rightarrow Steering Ratio =  \frac{\theta _{servo}}{\theta _{wheels}} = 1$

The steering ratio determines how much the wheels will turn in response to a given rotation of the servo. A higher steering ratio means that a smaller rotation of the steering wheel will result in a larger rotation of the wheels, providing a greater turning effect. Conversely, a lower steering ratio means that a larger rotation of the steering wheel is required to achieve the same wheel rotation. So, it can be described as a measure of the sensitivity and responsiveness of the steering system.

**2. Steering Wheel Torque**
 
Steering wheel torque T<sub>wheels</sub>
is the influence of servo motor force or moment applied on the steering. In other words, it refers to the force or moment applied to the wheels as a result of the torque output from the servo motor.
T<sub>wheels</sub> can be calculated by multiplying the applied force by servo motor F<sub>servo</sub> by the effective lever arm or steering arm length L<sub>eff</sub> which is the distance between the point where the force is applied and the axis of rotation of the steering wheel: 

$T_{wheel} =  F_{servo}  \times L_{eff}$

When interpreting the steering wheel torque, it should be noted that a higher steering wheel torque indicates that more force is needed from the servo motor to turn the steering wheel, providing a greater resistance to steering. Conversely, a lower steering wheel torque implies that less force is required, resulting in lighter and easier steering.

**3. Steering Wheel angle**

The steering wheel angle refers to the angle of rotation of the servo motor's output shaft, which is directly connected to the steering mechanism of the robot. Furthermore, it represents the angular displacement of the servo motor's output shaft from its neutral or reference position. 
This angle determines the orientation of the robot's steering mechanism and thus influences the robot's movement and direction.

$\theta_{wheel} =  \theta_{motor}  \times Steering Ratio$

---

### Differential Gear
Differential gear is a crucial component in our robot that allows the rear-wheels to rotate at different speeds at the same time while maintaining power distribution. The primary function of the differential gear is to enable the wheels on the same axle to rotate at different speeds when the robot is turning or when there is a difference in traction between the two wheels. It distributes torque from the engine to the wheels while allowing them to rotate at varying speeds. Our differential gear consists of several components:
- Ring Gear: A large gear attached to the axle shaft that receives power from the engine or the driveshaft.
- Pinion Gear: A smaller gear connected to the drive shaft, which meshes with the ring gear.
- Side Gears: Two gears that connect the differential to the axle shafts.
- Spider Gears: These gears are positioned between the side gears and allow the wheels to rotate at different speeds.

When the vehicle is moving in a straight line, the differential distributes torque equally to both wheels. However, when the vehicle turns, the inside wheel (closer to the center of rotation) needs to rotate at a slower speed than the outside wheel to prevent slippage. The differential allows this speed difference by allowing the side gears to rotate at different speeds while maintaining power distribution.
It is important to determine gear ratio before designing a differential gear, which involves understanding the specifications and characteristics of the motors, as well as the mechanical setup of the differential. To calculate the differential gear ratio, the ratio between the ring gear and the side gears should be determined. This can be done by counting the number of teeth on each gear:
   - The number of teeth on the ring gear (crown wheel). Let's call it $R$
   - The number of teeth on each side gear (sun gear). Let's call them $S1$ and $S2$
The differential gear ratio can be calculated as the ratio between the teeth on the ring gear and the teeth on one side gear. Since there are two side gears in the differential, the gear ratio is multiplied by 2:  

$Gear Ratio =  \frac{2  \times R}{ S_{1} } = \frac{2  \times R}{ S_{2} }$

The gear ratio represents the rotational speed and torque distribution between the two side gears of the differential. 
Once the gear ratio calculation is done, motor speed can be related to the wheel speed. Since the side gears are connected to the wheels, their speed is directly proportional to the wheel speed. The no-load speed of the motor N<sub>MotorNoLoad</sub> from the motor's specifications, which will enable us to calculate the wheel speed using the formula:

$N_{wheel} =  \frac{N_{MotorNoLoad}}{Gear Ratio}$

The wheel speed represents the rotational speed of the LEGO differential's side gears, which directly affects the vehicle's speed. However, the calculated values may need to be calibrated when experimenting them (Real world values may differ from the calculated ones due to multiple constraints that can’t be considered when doing the calculations). However, the results and objectives of our design showed no need to the calibration.

The following diagram represents the steps of calculating the $N_{wheel}$ with respect to the $N_{MotorNoLoad}$ and the $Gear Ratio$:

![Flow Chart](/Documentation/Images/Steering+Mechanism/Correct1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 20:Calculation flowchart of N_wheel, gear ratio and N_MotorNoLoad </i></p>

![Illustrated Differential Gear Box](/Documentation/Images/Steering+Mechanism/Correct2.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 21:Differential gear box </i></p>

--- 

##Motor Driver

A motor driver is crucial for controlling motors as it enables the safe handling and delivery of the required power to drive the motor, while also providing protection features to prevent damage in abnormal operating conditions. Furthermore, it offers precise control, speed, direction, and acceleration, so a driver are essential for applications that require accurate positioning or motion control as in our case. It offers bidirectional control, compatibility with different motor types, and optimize power usage for improved system efficiency. We tested two types of them in our robot.

![Controller_Driver_Motor](/Documentation/Images/PowerAndSense/Controller_Driver_Motor.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 22:Motor controlling open loop</i></p>

---

###Wheels

Lego wheels were used in the robot's design, their advantage comes from their efficiency and light weight. The table below clarify the specifications of the wheels.

| Lego Wheel Specifications (Part ID: 56908 / 41897) | Value                                               |
| ------------------------------------------------- | --------------------------------------------------- |
| Total diameter                                   | 56 mm (2.2'')                                       |
| Tire width                                       | 28 mm (1.1'')                                       |
| Rim diameter                                     | 43 mm (1.69'')                                      |
| Rim width                                        | 26 mm (1.02'')                                      |
| Weight                                           | 23g                                                 |
| Rim material                                     | ABS (Acrylonitrile Butadiene Styrene)               |
| Tire material                                    | SEBS (Styrene-Ethylene Ethylene-Butylene-Styrene)   |
<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Table 1: LEGO Wheels Specifications </i></p>

![LEGO Wheel](/Documentation/Images/Steering+Mechanism/Wheel.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 23:Lego wheels </i></p>

###Lego Axles
Using LEGO axles (4.8mm Diameter) in both differential gears mechanism and wheels connecting significantly improves the robot’s efficiency. LEGO axles provide precise rotational motion, and minimize friction and energy losses. It is made of PA (Polyamide) which is a type of material that can withstand high loads and impacts. This means it’s suitable for elements that need to be tough and able to interact with each other, ensuring smooth power transmission and prolonged operation, for example, gearwheels and different connectors.

![Steering Axle](/Documentation/Images/Steering+Mechanism/Steering.jpg)
<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 24: LEGO Axle </i></p>

---

###Coupler Design:
Power transmission plays a critical role in ensuring efficient and reliable movement. We utilized a coupler to transfer torque from a Jsumo DC motor to a LEGO axle and also another coupler to transmit torque from a servo motor to the steering LEGO axle. This solution combines the flexibility and customization of 3D manufacturing with the versatility and precision of LEGO components, resulting in a robust and seamless power transmission mechanism. The coupler acts as the mediator between motors and the LEGO axles, seamlessly connecting both components while maintaining torque transfer efficiency. The design of the coupler is carefully engineered to fit snugly onto the motor shaft and securely attach to the LEGO axle, ensuring optimal power transmission without slippage or loss. This design can be produced using 3D Printers or CNC Cutting Machine.

**The figure shows the design of the coupler:**
![3D printed coupler](/Documentation/Images/couplers/coupler.jpg)
<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 25: Coupler Drawings (extracted from CATIA V5)  </i></p>


## Power & Sense Management <a id="PowerandSenseManagement"></a>

We used 2 Li-poly RC Batteries (12V – 2200mAh) as a source of power. The first one supplies
the Arduino Board, Sensors, Servo motor, and DC motor; the second one supplies the Raspberrypi microprocessor. The robot is power supplied by two batteries as we mentioned before. The
   distribution of power is done by Three RioRand step-down DC-DC Converter that take their input
   from the batteries directly (12V input) and convert this input to (5V output). The first one supplies
   the raspberry pi microprocessor and its cooling fan. The second gives supply to a “bridge” which
   in turn supply the sensors and the gyroscope. The third supplies the servo motor. The DC motor
   is supplied via BTS motor driver that’s supplied directly from the batteries.
   We used 2 Li-poly RC Batteries (12V – 2200mAh) as a source of power. The first one supplies 
   the Arduino Board, Sensors, Servo motor, and DC motor; the second one supplies the Raspberry
   pi microprocessor. The robot is power supplied by two batteries as we mentioned before. The
   distribution of power is done by Three RioRand step-down DC-DC Converter that take their input
   from the batteries directly (12V input) and convert this input to (5V output). The first one supplies
   the raspberry pi microprocessor and its cooling fan. The second gives supply to a “bridge” which
   in turn supply the sensors and the gyroscope. The third supplies the servo motor. The DC motor
   is supplied via BTS motor driver that’s supplied directly from the batteries.

### Electronic Components
- **Arduino Mega 2560:** [[2]](https://store.arduino.cc/products/arduino-mega-2560-rev3) The Arduino Mega 2560 is an open-source microcontroller board based on the ATmega2560 microcontroller. It is one of the most popular boards in the Arduino family due to its versatility and extensive I/O capabilities. The Mega 2560 is designed to offer a large number of digital and analog inputs and outputs, making it suitable for complex projects that require multiple sensors, actuators, and communication interfaces. The key features and specifications of the Arduino Mega 2560:

   1. Microcontroller: The Mega 2560 is powered by the ATmega2560 microcontroller, which has 256KB of flash memory for storing code, 8KB of SRAM for data storage, and 4KB of EEPROM for non-volatile storage.

   2. Digital I/O Pins: The board offers 54 digital I/O pins, of which 15 can be used as PWM outputs. These pins can be used for connecting various components such as sensors, switches, LEDs, and other digital devices.

   3. Analog Inputs: The Mega 2560 has 16 analog inputs, allowing you to read analog signals from sensors and other devices. These inputs have a 10-bit resolution, providing 1024 different voltage levels.

   4. Communication Interfaces: The board supports multiple communication interfaces, including UART (Universal Asynchronous Receiver/Transmitter), SPI (Serial Peripheral Interface), and I2C (Inter-Integrated Circuit). These interfaces enable communication with other devices such as sensors, displays, and modules.

   5. USB Interface: The Mega 2560 features a USB interface that allows it to be connected to a computer for programming and serial communication. It can appear as a virtual serial port for easy integration with software development environments.

   6. Operating Voltage: The board operates at 5V, making it compatible with a wide range of sensors, modules, and other components.

   7. Memory Expansion: The Mega 2560 supports external memory expansion through an optional SD card slot, allowing you to store larger amounts of data or log sensor readings.

   8. Compatibility: The Mega 2560 is fully compatible with the Arduino software and programming language, making it easy to get started with coding and prototyping.

   9. Shield Compatibility: The board is designed to be compatible with Arduino shields, which are additional modules that can be stacked on top of the board to extend its functionality. This allows you to easily add features such as Wi-Fi, Bluetooth, motor control, and more.  

     ![Ardunio](/Documentation/Images/PowerAndSense/Arduino.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 26: Arduino Mega2560</i></p>

- **Raspberry pi 4 Model B:** [[3]](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) The Raspberry Pi 4 Model B is a popular single-board computer (SBC) developed by the Raspberry Pi Foundation. It is the fourth generation of the Raspberry Pi series and offers significant improvements over its predecessors. The Raspberry Pi 4 Model B is designed to be a versatile and affordable platform for various applications, including education, prototyping, home automation, media centers, and more. The key features and specifications of the Raspberry Pi 4 Model B:
   1. Processor: The board is powered by a Broadcom BCM2711 quad-core Cortex-A72 (ARMv8) 64-bit system-on-a-chip (SoC) running at 1.5 GHz. This processor provides a significant performance boost compared to previous Raspberry Pi models, enabling smoother multitasking and faster execution of applications.

   2. Memory: The Raspberry Pi 4 Model B is available in different memory configurations, including 2GB, 4GB, and 8GB LPDDR4 RAM options. The increased memory capacity allows for better performance, especially when running resource-intensive applications or multitasking.

   1. GPU: It features a VideoCore VI GPU, which supports OpenGL ES 3.x and 4Kp60 hardware decode of HEVC video. The GPU enables multimedia applications, gaming, and hardware-accelerated video playback.

   1. Connectivity: The board offers improved connectivity options, including:
     * Gigabit Ethernet: Provides fast wired network connectivity.

     * Dual-band Wi-Fi: Supports 2.4 GHz and 5 GHz wireless networks, offering improved wireless performance.
     * Bluetooth 5.0: Enables wireless connectivity with Bluetooth-enabled devices.
     * USB Ports: The Raspberry Pi 4 Model B has two USB 2.0 ports and two USB 3.0 ports, allowing for easy connection of peripherals such as keyboards, mice, external drives, and more.
   5. Video and Display: The Raspberry Pi 4 Model B supports dual-monitor output with resolutions up to 4K. It features two micro HDMI ports, allowing you to connect two displays simultaneously. The board also has a MIPI DSI display port and a MIPI CSI camera port for connecting touchscreens and cameras.

   1. Storage: It includes a microSD card slot for the operating system and data storage. Additionally, it has two USB 3.0 ports and two USB 2.0 ports, which can be used to connect external storage devices such as hard drives or USB flash drives.

   1. GPIO Pins: The board maintains compatibility with previous Raspberry Pi models and provides 40 GPIO (General Purpose Input/Output) pins, which allow for the connection of various sensors, actuators, and other electronic components.

   1. Operating System: The Raspberry Pi 4 Model B is compatible with a wide range of operating systems, including Linux distributions such as Raspbian (now known as Raspberry Pi OS), Ubuntu, and others. This flexibility enables developers to choose the OS that best suits their needs and leverage the extensive software ecosystem available for the Raspberry Pi. 
    
     ![RaspberryPi](/Documentation/Images/PowerAndSense/RaspberryPi.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 27: RaspberryPi</i></p>

- **Raspberry pi camera:** [[4]](https://www.raspberrypi.com/products/camera-module-v2/) The Raspberry pi camera was used to discover the playfield of the robot and to detect the color of the pillars existed in front of the vehicle. 

     ![PiCamera](/Documentation/Images/PowerAndSense/Camera.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 28: RaspberryPi Camera</i></p>

- **BTS7960 Motor Driver:** [[5]](https://uk.farnell.com/c/sensors-transducers/sensors/ultrasonic-sensors)
    The BTS7960 Motor Driver is a popular dual H-bridge motor driver module that allows you to control the speed and direction of DC motors. It is commonly used in robotics, automation, and other projects that require precise motor control.
    The key features and specifications of the BTS7960 Motor Driver:

    **i.H-Bridge** Configuration: The BTS7960 Motor Driver utilizes an H-bridge configuration, which consists of four power MOSFETs (Metal-Oxide-Semiconductor Field-Effect Transistors) arranged in pairs. This configuration enables bidirectional control of the motor, allowing you to control its rotation in forward and reverse directions.

    **ii.Motor Compatibility:** The motor driver module is designed to work with DC motors and can handle a wide range of motor voltages, typically between 5V and 27V. The maximum continuous current per channel is typically around 43A, making it suitable for driving high-power motors.

    **iii.Current Sensing:** The BTS7960 Motor Driver features built-in current sensing capabilities. It has two current sense pins that allow you to monitor the current flowing through each motor channel. This feature can be useful for various purposes, such as implementing motor current protection or feedback control.

    **iiii.PWM Control:** The motor driver supports Pulse Width Modulation (PWM) control for speed regulation. By varying the duty cycle of the PWM signal, you can adjust the motor's speed precisely. The module accepts a PWM input signal from a microcontroller or any other PWM source.

    **iv.Overcurrent and Overtemperature Protection:** The BTS7960 Motor Driver incorporates protection mechanisms to safeguard both the driver and the motor. It includes built-in overcurrent and overtemperature protection circuits that can help prevent damage to the motor driver module and the connected motor.

    **vi.Input and Output Connections:** The motor driver module typically has separate input pins for controlling the motor direction and speed. These control pins can be connected to the digital output pins of a microcontroller or other control devices. The motor connections are made to the driver's output terminals, allowing you to connect the DC motor.

    **vii.Heat Dissipation:** Due to its high current handling capacity, the BTS7960 motor driver may generate heat during operation. To dissipate heat effectively, the module often includes a heat sink or a mounting hole for attaching an external heat sink.

    **viii.Compatibility:** The BTS7960 Motor Driver can be used with various microcontrollers, such as Arduino, Raspberry Pi, or any other microcontroller capable of generating the necessary control signals for the driver module.
    
     ![BTS7960](/Documentation/Images/PowerAndSense/BTS7960.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 29: BTS7960 Motor Driver</i></p>

- **L298N Motor Driver:**

    The L298N Motor Driver is another widely used motor driver module that provides control over the speed and direction of DC motors. It also utilizes an H-bridge configuration, consisting of four power transistors arranged in pairs, enabling bidirectional control of the motor's rotation.

    **i.H-Bridge Configuration:** The L298N utilizes a dual H-bridge motor driver that allows speed and direction control of two DC motors at the same time, this configuration
    contains four switching elements, transistors or MOSFETs, with the motor at the center. It also has two switches that can be used to control the motor rotation in either direction. Moreover, the input pins of the L298N are used for controlling the rotation direction of the motor A and B, and by controlling these pins, the switches of the H-Bridge inside the L298N IC are controlled.

    **ii.Motor Compatibility:** The L298N Motor Driver is designed to work with DC motors and can handle a wide range of motor voltages, it is suitable for driving medium-power motors.

    **iii.Current Sensing:** The L298N Motor Driver does not have built-in current sensing capabilities, but external current sensing circuitry can be implemented separately if current monitoring is required.

    **vi.PWM Control:** The L298N Motor Driver supports PWM control for regulating the motor's speed. By adjusting the duty cycle of the PWM signal, the motor's speed can be precisely controlled.

    **v.Overcurrent Protection:** The L298N Motor Driver incorporates overcurrent protection mechanisms to prevent damage to the driver and the connected motor. However, it does not typically include built-in overtemperature protection.

    **vi.Input and Output Connections:** The motor driver module has separate input pins for controlling the motor direction and speed. These control pins can be connected to the digital output pins of a microcontroller or other control devices, while the motor connections are made to the driver's output terminals.

    **vii.Heat Dissipation:** The L298N Motor Driver may generate heat during operation due to its current handling capacity. It often includes a heat sink or a mounting hole for attaching an external heat sink to dissipate heat effectively.

    **viii.Compatibility:** The L298N Motor Driver is compatible with various microcontrollers, including Arduino, Raspberry Pi, and other microcontrollers capable of generating the necessary control signals for the driver module.
  
  ![L298N](/Documentation/Images/PowerAndSense/L298N.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 30: L298N Motor Driver</i></p>
- **Comparison between L298N and BTS7960:**

    We tested both circuits and evaluated their performance on the robot. However, BTS7960 Motor Driver showed noticably better results, on the other hand, L298N didn't meet the requirements of our robot which resulted in overheated circuit and not enough current input current to the JSumo DC Motor. Here are the key differences between both motor drivers:
    
    1. **Current Handling:** The BTS7960 Motor Driver has a higher current handling capacity compared to the L298N Motor Driver, making it more suitable for driving high-power motors (J-Sumo Motor in our case).

    2. **Current Sensing:** The BTS7960 Motor Driver includes built-in current sensing pins, allowing for easy monitoring of motor current. On the other hand, the L298N Motor Driver does not have this feature, requiring external circuitry for current sensing.

    3. **Overtemperature Protection:** The BTS7960 Motor Driver incorporates overtemperature protection, which helps safeguard the driver and the motor. The L298N Motor Driver does not typically include this protection mechanism.

    4. **Voltage Range:** The L298N Motor Driver has a wider voltage range compared to the BTS7960.


- **3 RioRand LM2596 step-down DC-DC Converter:** [[6]](https://www.amazon.com/RioRand-LM2596-Converter-1-23V-30V-5Pcs-LM2596/dp/B008BHB4L8) The RioRand LM2596 is a popular step-down DC-DC converter module based on the LM2596 switching regulator integrated circuit. It is commonly used to convert higher DC voltages to lower, more stable voltages for various electronic applications. The LM2596 module offers efficient and adjustable voltage regulation, making it widely used in projects requiring reliable power supply solutions.

  The key features and specifications of the RioRand LM2596 step-down DC-DC converter:

   1. Conversion Efficiency: The LM2596 module utilizes a switching regulator topology, which provides high conversion efficiency compared to linear regulators. It can achieve conversion efficiencies of up to 90% or more, depending on the input and output voltage differentials.

   2. Input Voltage Range: The LM2596 module can accept a wide range of input voltages, typically between 4.5V and 40V DC. This flexibility allows it to be used with various power sources, such as batteries, power adapters, or other DC power supplies.

   3. Adjustable Output Voltage: One of the key features of the LM2596 module is its ability to provide adjustable output voltage. By adjusting the onboard potentiometer or using an external voltage divider, you can set the desired output voltage within the module's specified range. The output voltage can typically be adjusted from around 1.25V to 37V DC.

   4. Output Current: The LM2596 module can handle a maximum output current of around 2A or more, depending on the specific module variant and configuration. This current rating determines the maximum load current that the module can reliably supply.

   5. Protection Features: The LM2596 module often includes built-in protection features to ensure the safety and longevity of the circuit. These features may include overcurrent protection, thermal shutdown, and input/output overvoltage protection.

   6. Heat Dissipation: Like other power electronics components, the LM2596 module may generate heat during operation. To dissipate heat effectively and maintain optimal performance, the module typically includes a heat sink or a mounting hole for attaching an external heat sink.

   7. Input and Output Connections: The LM2596 module typically has screw terminals or pin headers for easy connection to the input and output voltage sources. The input voltage is connected to the higher voltage source, while the output voltage is connected to the load or the circuit that requires the lower voltage supply.

   8. Applications: The LM2596 module finds applications in a wide range of projects, including electronics prototyping, DIY projects, power supply modules, battery charging systems, LED lighting, and more. Its adjustable output voltage and high efficiency make it a versatile choice for powering various electronic devices and circuits. 
 
 ![Dc2Dc converter](/Documentation/Images/PowerAndSense/DcDcConv.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 31: RioRand  LM2596 step-down DC-DC Converter</i></p>

* **Servo Motor:** [[7]](https://www.towerpro.com.tw/product/sg90-7/)
  + Torque: 1,6kgcm
  + Stall current 650±80mA
  + Voltage 4,8 V - 7,2 V
  + Speed 0.12 sec / 60 degrees (4.8V)
  + Gears Nylon
  + Length 22 mm
  + Width 11,5 mm
  + Height 22,5 mm Weight 16 g 

     ![Servo](/Documentation/Images/PowerAndSense/Sg.webp =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 32: Servo Motor</i></p>
- **Li-Po RC Batteries 12V:** [[8]](https://robocraze.com/products/11-1v-2200mah-25c-li-po-battery) Lithium Polymer (Li-Po) batteries are a type of rechargeable battery that are widely used due to their high energy density, lightweight, and ability to deliver high currents. It provides a good balance between capacity and weight.


  | Battery Parameter     | Value                                 |
  | --------------------- | ------------------------------------- |
  | Model                 | ZOP Power 11.1V 2200mAh 30C           |
  | Capacity              | 2200mAh                               |
  | Size                  | 28x34x116mm                           |
  | Plug Style            | XT60 Plug                             |
  | Weight                | 140g                                  |
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Table 2: LiPo Battery Specifications </i></p>

  ![Lithium Polymer (Li-Po)](/Documentation/Images/PowerAndSense/Lipo(1).jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 33: LiPo 2800mAh Battery</i></p>

- **J Sumo DC Motor:** [[1]](https://www.jsumo.com/jsumo-titan-dc-gearhead-motor-12v-1000-rpm-hp)
  - The diameter of the motor is 37mm. Length is 75mm.
  - 6mm diameter shaft section, shaft length 17 mm.
  - 6 M3 screw holes on the front side are available.
  - No load current is 240ma, and stall current is 5.9 Ampere.
  - All Gearhead gears are metal.
  - Motor’s stall torque is 7.5 kg-cm.

     ![Dc](/Documentation/Images/PowerAndSense/jSumo.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 34: jSumo Dc Motor</i></p>

- **MPU 6050:** [[9]](https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module) The MPU-6050 is a popular and widely used integrated circuit (IC) that combines a 3-axis gyroscope and a 3-axis accelerometer in a single package. It is commonly used in various applications such as motion sensing, orientation tracking, gesture recognition, and stabilization control systems.

  The key features and specifications of the MPU-6050:

    1. 3-Axis Gyroscope: The MPU-6050 includes a 3-axis gyroscope, which measures angular velocity around three axes (X, Y, and Z). It provides accurate motion sensing capabilities and can be used to detect rotational movements.

    2. 3-Axis Accelerometer: The MPU-6050 also incorporates a 3-axis accelerometer, which measures linear acceleration along the three axes (X, Y, and Z). It enables the detection of changes in velocity and the determination of tilt or inclination.

    3. Digital Motion Processing: The MPU-6050 features an onboard digital motion processor (DMP) that offloads motion processing tasks from the main microcontroller. The DMP performs sensor fusion, combining data from the gyroscope and accelerometer to provide accurate and reliable motion tracking information.

    4. I2C Interface: The MPU-6050 communicates with the microcontroller or other devices using the I2C (Inter-Integrated Circuit) protocol. It has an I2C interface that allows for easy integration and control with various microcontrollers and development boards.

    5. Motion Detection Interrupts: The MPU-6050 includes built-in motion detection capabilities and supports programmable interrupts. This allows the IC to generate interrupts to the microcontroller when predefined motion or orientation thresholds are met, enabling efficient and responsive operation.
 
    6. Low Power Consumption: The MPU-6050 is designed to operate with low power consumption, making it suitable for battery-powered applications and devices with power constraints.

    7. Temperature Sensor: The IC incorporates an onboard temperature sensor, providing accurate temperature measurements in addition to motion and orientation data.

    8. Applications: The MPU-6050 is widely used in applications such as robotics, drones, gaming controllers, virtual reality (VR) and augmented reality (AR) devices, inertial navigation systems, motion-controlled user interfaces, and many other projects where motion sensing and orientation tracking are required. 

     ![Mpu6050](/Documentation/Images/PowerAndSense/mpu6050.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 35: MPU6050</i></p>

- **Cooling Fan:** The cooling fan is used to cool down the temperature of the Raspberry pi microprocessor.

     ![Cooling Fan](/Documentation/Images/PowerAndSense/CoolingFan.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 36: Cooling Fan</i></p>

- **UltraSonic Sensors:** [[10]](https://uk.farnell.com/c/sensors-transducers/sensors/ultrasonic-sensors)
  Ultrasonic sensors are electronic devices that use ultrasonic waves to measure distance or detect objects. They work based on the principle of echolocation, similar to how bats navigate in the dark. Ultrasonic sensors emit high-frequency sound waves (typically above the range of human hearing) and then receive the reflected waves to determine the distance or presence of objects.

  The key characteristics and applications of ultrasonic sensors:

    1. Distance Measurement: Ultrasonic sensors are commonly used for non-contact distance measurement. They typically emit ultrasonic pulses and measure the time it takes for the sound waves to bounce back after hitting an object. By knowing the speed of sound in the medium (usually air), the sensor can calculate the distance to the object.

    2. Object Detection: Ultrasonic sensors can detect the presence or absence of objects within their range. When an object is detected, the sensor receives the reflected sound waves, and based on the intensity or time delay of the received signal, it can determine if an object is present or not.

    3. Range and Accuracy: Ultrasonic sensors can have varying range capabilities, typically ranging from a few centimeters to several meters. The accuracy of distance measurement depends on factors such as the sensor's resolution, beam angle, and environmental conditions.

    4. Ultrasonic Transducer: The core component of an ultrasonic sensor is the transducer, which converts electrical energy into ultrasonic sound waves and vice versa. It consists of a piezoelectric crystal that vibrates when an electrical current is applied, generating the ultrasonic waves.

    5. Beam Pattern: Ultrasonic sensors emit sound waves in a specific beam pattern, usually conical or cylindrical. The beam angle determines the width of the detection area. Narrower beam angles provide more focused sensing, while wider angles cover a larger area but with reduced accuracy.

    6. Environmental Considerations: Ultrasonic sensors are affected by environmental factors such as temperature, humidity, and air turbulence. These factors can influence the speed of sound and affect the accuracy of distance measurements.

    7. Applications: Ultrasonic sensors are widely used in various fields, including industrial automation, robotics, automotive, security systems, level monitoring, parking assistance, liquid flow measurement, and presence detection in consumer electronics. 

The 10 Sensor were mounted as the following:
 - `L1` Sensor Left Front (Closer to the frontend of the robot).
 - `L2` Sensor Left Back (Closer to the backend of the robot).
 - `R1` Sensor Right Front (Closer to the Frontend of the robot).
 - `R2` Sensor Right Back (Closer to the backend of the robot).
 - `F` Sensor Frontal Sensor (In the middle of the frontend of the -obot).
 - `B` Sensor Rear Sensor (In the middle of the backend of the robot).
 - `BL` Sensor Back Left Sensor - mounted on the edge of the vehicle and tilted by 45 counterclockwise degrees about the x-axis of the robot.
 - `BR` Sensor Back Right Sensor - mounted on the edge of the vehicle and tilted by 45 clockwise degrees about the x-axis of the robot.
 - `FL` Sensor Front Left Sensor - mounted on the edge of the vehicle and tilted by -45 counterclockwise degrees about the x-axis of the robot.
 - `FR` Sensor Front Right Sensor - mounted on the edge of the vehicle and tilted by -45 clockwise degrees about the x-axis of the robot.

     ![ultrasonic sensor](/Documentation/Images/PowerAndSense/HC-SR04.jpg =300x)
  <p Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 37: Ultra Sonic Sensor - HC-SR04</i></p>

###Electrical Components Current Needs And Overall Circuit Usage

Accurate knowledge of the electrical component requirements is a crucial factor to be considered. Both theoritical and experimental information should be gathered to investigate the properties of the electrical circuit, this can be reduced to the following reasons:

- Optimal Power Supply Design:
Understanding the current needs of the robot's electrical components allows for the design of an appropriate power supply system, this helps in selecting a power source that can provide the required currents at the correct voltage levels so that the system's stability and performance can be ensured.

- Battery Life Optimization:
Since Lipo Batteries were used in the robot, knowing the current needs of the components is crucial for optimizing battery life.In results, Estimating accurately the current draw during different operational states, power-saving strategies can be implemented, such as allocating convenient batteries to the needed components. Additionally, this extends the robot's operating time and reduces the frequency of battery replacements or recharging.

- Heat Dissipation and Cooling:
Some electrical components may suffer from overheating due to its continous high current draw, so accurate knowledge of the overall circuit consumption aids in determining the heat generated by the components, which in sequence helps in implementing effective cooling mechanisms to prevent overheating, which can lead to performance degradation or even component failure. For example, we noticed that Raspberry Pi heat's increases noticably after long time of working, so we added a cooling fan in addition to heat sink wich helped avoiding overheating problem.

- Safety Considerations:
Understanding the overall circuit consumption is surely essential from a safety standpoint. By ensuring that the power supply system can handle the maximum expected current draw, the risk of electrical failures, such as voltage drops or circuit damage, can be mitigated, which also prevents potential hazards to both the robot and its environment.

- Cost Optimization:
When the electrical needs are known for each component, costs can be reduced without compromising performance or reliability by selecting components that meet the necessary specifications.

The properties of the robot's circuit can be understood in the table and figure below:

UltraSonic Sensor: The input current for an Ultrasonic Sensor can vary depending on the specific model and operating conditions. 
Typically, it ranges from a few milliamperes (mA) to around 15 mA. For more information check [here](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiiv5HGo4qCAxXP2wIHHajnCG0QFnoECBwQAQ&url=https%3A%2F%2Fcdn.sparkfun.com%2Fdatasheets%2FSensors%2FProximity%2FHCSR04.pdf&usg=AOvVaw1iQg0OJ6MFfs9MrkZYGAB4&opi=89978449)

MPU6050 Gyroscope Sensor: The input current for an MPU6050 Gyroscope Sensor is about 36 mA. For more information check [here](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

Raspberry Pi Model B 4: The input current for a Raspberry Pi Model B 4 can vary depending on the connected peripherals and the workload. Typically, it requires around 2.5 A, but it is recommended in our case to use a power supply capable of providing 3A for stable operation. For more information check
[here](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/).

Arduino Mega: The input current for an Arduino Mega board is typically around 50mA when idle. However, the current consumption can increase when using additional peripherals or when running more complex programs as the case in our robot.
For more information check [here](https://www.mantech.co.za/datasheets/products/A000047.pdf) and [here](https://store.arduino.cc/products/arduino-mega-2560-rev3).

Servo Motor SG Tower Pro: The input current for a Servo Motor can vary depending on the load and the specific model. Typically, it ranges from  0.5A – 2A, depending on the torque and speed requirements.
For more information check  [here](https://www.towerpro.com.tw/product/sg90-7/#:~:text=The%20operation%20current%20for%20our,4.8V%20–%206V%20are%20fine.)

Raspberry pi camera: The input current for a Raspberry Pi Camera Module can vary depending on the model and the camera settings. Typically, it requires around 250-300mA.

BTS7960 Motor Driver: The input current for a BTS7960 Motor Driver depends on the motor being driven and the load conditions. It can range from a few milliamperes (mA) to several amperes (A), depending on JSumo motor's state.

| Component                  | Input Current (mA) |
|----------------------------|-------------------|
| Ultrasonic Sensor         | 15             |
| MPU6050 Gyroscope         | 3.6            |
| Raspberry Pi Model B 4    | 3000                 |
| Arduino Mega              | 500                |
| Servo Motor SG Tower Pro  | 500-2000               |
| Raspberry pi camera  | 250 - 300               |


###Circuit ground

A common ground in a circuit is essential for maintaining accurate voltage measurements, ensuring reliable signal transmission, and mitigating signal noise and interference. It provides a consistent voltage reference for all components, simplifies circuit design and troubleshooting, and aids in maintaining signal integrity throughout the circuit. By establishing a shared reference point, the common ground enables effective communication between components and promotes optimal circuit operation.

----

##Obstacle Management <a id="ObstacleManagement"></a>

Obstacle management is a crucial challenge that should be driven by robust algorithms, which offers precision without the loss of the speed. We developed our algoritms based on the previous requirements, and thus achieved noticeably reliable performance.  


###Computer Vision <a id="ComputerVision"></a>

The integration of computer vision in latest techniques is considered an essential technology in robotics and automation. This can be seen especially in self-driving cars, where enabling the vehicle to see is essential, which means that it can perceive and understand the environment. By employing CV (computer vision) techniques, the robot in our task can gather valuable information from its surrounding, make informed decisions, and perform the required tasks. In this context, the isolation of red and green pillars is critical to prevent collisions and ensure the robot's path remains obstacle-free. However, since the computer vision function is only identifying red and green pillars of same size, the complexity can be reduced to color detection problem without the need for object detection. After determining the required CV algorithm, among the various color detection approaches available, such as HSL (Hue, Saturation, Lightness), HSB (Hue, Saturation, Brightness), RGB (Red, Green, Blue), and HSV (Hue, Saturation, Value), we have chosen to utilize the HSV color space, because in comparison to these color spaces:

- The HSV color space offers distinct advantages over other color spaces for our specific requirements. HSV separates color information from both brightness and saturation. This separation allows for better adaptation to different lighting conditions, making the HSV color space more robust and reliable for color detection tasks.
- While RGB is widely used and suitable for many applications, it presents challenges in color-based segmentation and sensitivity to lighting variations. The complex color manipulation in RGB makes it less suitable for our specific needs. On the other hand, the HSV color space simplifies the extraction and separation of specific colors from images, making it ideal for efficient color-based segmentation.

While the RGB color space is widely used and suitable for many applications, it was deemed less suitable for the specific requirements of image processing and color detection. RGB's complex color manipulation, challenges in color-based segmentation, and sensitivity to lighting variations were the primary factors that influenced our decision to opt for the HSV color space.

####Pre-Processing: <a id="Pre-Processing"></a>

Before isolating the pillars to extract information, it was necessary to apply pre-processing on the captured images to improve the overall effectiveness of subsequent isolation. Preprocessing helps to eliminate unwanted artifacts and enhance relevant image features leading to more accurate and reliable segmentation results. It allows to prepare the image data in a way that maximizes the CV algorithm's performance, ensuring robust color detection and accurate boundary delineation. Our pre-processing operations contained applying a linear transformation to the pixel intensity values of the images. We used alpha and beta parameters, the alpha parameter controls the contrast adjustment, determining how much the pixel values are scaled. A larger alpha value increases the contrast, making the dark regions darker and the bright regions brighter. Additionally, the beta parameter represents the brightness adjustment, determining the amount of brightness added to the image. By adjusting these parameters, they can be used to modify the image's appearance, enhancing or reducing the contrast and brightness levels. Moreover, we applied Gaussian filtering. By averaging the pixel values in the neighborhood, with more emphasis on the nearby pixels according to their proximity to the center. it effectively reduced high-frequency noise and smoothed out details. We also applied erosion to shrink the boundaries of objects in an image in order to remove small details and smoothen the image. Finally, Dilatation was applied to expand the boundaries of objects to enhance the size and shape of objects.
 
![Pre-Processing flow chart](/Documentation/Images/ComputerVision/CV2.png)
<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 38:Pre-Processing flow flowchart </i></p>

####Masks:

To isolate the color of green and red pillars we applied three masks on the pre-processed image. These masks black out all pixels that are not in the range of the specified HSV values. HSV masks are binary images that are used to isolate specific colors or ranges of colors in an image based on the HSV color space. These masks are created by setting pixels within the desired color range to white (255) and pixels outside the range to black (0). HSV masks are commonly used in various computer vision applications, such as color-based object detection, image segmentation, and tracking. The steps of making the masks are:
1. Convert the input image from the RGB color space to the HSV color space.
1. Define the lower and upper thresholds for the desired color range in terms of hue, saturation, and value values.
1. Iterate through each pixel in the HSV image and check if it falls within the specified color range.
1. For pixels within the range, set the corresponding pixel in the mask image to white (255); otherwise, set it to black (0).
1. The resulting mask image will have white pixels representing the desired color range and black pixels representing all other colors.

HSV values in our work were as follows: (([0,100,50] to [0,255,255] and [163,100,54] to [179,255,255] for red due to the cylindrical shape of the hue values) – (([43, 50, 45] to [43, 50, 45] for green)). 

####Image Thresholding:
Thresholding is a technique used to convert a grayscale or single-channel image into a binary image, where pixel values are classified into two categories based on a specified threshold value.
The general syntax for using OpenCV library thresholding:

*ret, threshold = cv2.threshold(src, thresh, maxval, type)**
* src: The source image, which should be a single-channel (grayscale) image.
* thresh: The threshold value used for classifying pixel values.
* maxval: The maximum value assigned to pixels that exceed the threshold.
* type: The thresholding type, which determines the specific algorithm used for thresholding. 

Common types include cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV, cv2.THRESH_TRUNC, cv2.THRESH_TOZERO, and cv2.THRESH_TOZERO_INV.

Return values:
1. ret: The threshold value that was used (sometimes automatically calculated).
2. threshold: The resulting binary image after applying the thresholding operation.

The cv2.threshold function compares each pixel value in the source image with the threshold value. Pixels with values greater than the threshold are assigned the maxval value, while pixels with values below the threshold are set to 0 or a lower value (depending on the thresholding type).
####Contouring:

Contouring in OpenCV and computer vision refers to the process of detecting and representing the boundaries of objects or shapes in an image. Contours are widely used in computer vision for various tasks, such as object detection, shape analysis, motion tracking, and image segmentation.

####What pillar is the closest pillar to the vehicle?
The decision of what pillar is closest  to the vehicle is made by many selection processes as following and for each contour in the image:
1. Finding the moments (specifically the centroid) of the contour.
2. Finding the bounding rectangle about the contour.
3. Calculating the area of the contour using cv2.contourArea function.
4. Calculating the mean area of the two previous contours(Since the first contour is a rectangle, it may not give an accurate area with some orientations of the camera. Additionally, the second contour may contain noise and not thus don’t give the correct area)
5. Taking the largest area contour as the selected one.
These steps are applied on the red and green masks.
It should be noted that the orientation of the used camera (Raspberry pi camera) maintain a range of view that allow to detect more than one pillar, more specifically, when the robot is in the corner section, the pillars in the end of the next section can be seen on the camera, but without objects from outside (We cropped the images to ensure that no outer objects are evident in the image).
 
 ![Area Calculation](/Documentation/Images/ComputerVision/CV1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 39: Area calculation flowchart </i></p>

####What is the distance between the pillar and the vehicle?
After finding the largest contour we can find the distance between the vehicle and the pillar using a mathematical expression found by a curve fitting model that was built on experimental data of the contour area and the actual distance. The data can be found in Appendix **:
 
 ![Area Calculation](http://www.sciweavers.org/tex2img.php?eq=distance%20%3D%20%20%20%5Calpha%20%20%20%5Ctimes%20%20%20Area%5E%7B%20%5Cbeta%20%7D&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

####How is CV used in the robot?

Two algorithms were developed and tested for CV processing, One algorithm *"Persistent Vision"* operates continuously, processing incoming images in real-time and promptly relaying the results. The other algorithm "Event Driven Vision", on the other hand, remains dormant until it receives a start signal, and sends the results once only.

**1 - Persistent Vision**  <a id="PersistentVision"></a>

The Persistent Vision algorithm continuously processes camera input images and performs a series of pre-processing operations detailed in [Pre-Processing](#Pre-Processing). It utilizes the HSV color space to isolate colors and identifies the region of interest, which corresponds to the red and green pillars. Subsequently, it calculates the area of all contours present in the image and selects the closest one. Based on the measured distance,  the algorithm determines the state of the pillar. Once the colsest pillar to the vehicle is identified, the raspberry pi sends a signal composed of single character via serial connection to the microcontroller. This signal consists of one of the following letters that define the state of the pillar: "G," "R," "g," "r," or "N". These letters indicate the presence of: a close green pillar (distance < 50cm),  close red pillar (distance < 50cm), far green pillar (distance > 50cm), far red pillar (distance > 50cm), or no green or red pillar, respectively.

  ![Persistent Vision](/Documentation/Images/ComputerVision/PersistentVision.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 40:Persistent Vision algorithm flowchart </i></p>


**2 - Event Driven Vision** <a id="Event DrivenVision"></a>

In the Event Driven Vision algorithm, processing occurs once a signal is received from the Arduino. The Arduino sends this signal at the beginning of each section, after the robot has stopped and assumed the correct position. The algorithm follows the same steps as before but with some modifications. It takes all pillars contours that are present in the scene (camera input images) and applies the following procedures. We will refer to the section the robot is entering as the "current section" and the following section as the "next section":

1- Count the number of pillars within the maximum range present in the image. Pillars located beyond the end of the current section are disregarded. This comes from the fact that they do not impact the robot's movement decision in the current section.

2- Once the number of pillars is determined, their information is stored in an array that includes the area, X and Y coordinates of the contour centroid, color, and distance for each contour.

3- There are three cases to consider:

- If there are three stored pillars, the array is sorted based on the area. This situation arises when there are two pillars in the current section and one pillar is present at the start of the next section. In a such case, the HSV color isolation may fail to ignore the farthest pillar, leading to inaccurate distance calculation and the inclusion of unwanted pillars. The contour centroids are used to identify the pillars within the borders of the current section. This is based on the understanding that when the moving direction (CW or CCW) is known, the x-coordinates of contours centroids can be used to determine which pillar is unwanted.
- If there are only two stored pillars, the array is sorted directly based on the centroids. The presence of a pillar at the beginning of the current section can be easily determined through distance, there are two subcases to consider. In the first subcase (Existence of a pillar at the start of the current section), the other pillar is taken into account. In the contrary subcase, where it is impossible for two pillars to exist in the same section when neither is at the beginning of the current section, only the closest pillar is considered.
- The simplest case involves the detection of only one pillar. In this scenario, the decision is made directly based on the color alone.

After finishing the processing, only one signal is sent to arduino and image capturing with processing will stop, it will wait until a signal from the arduino arrives again. The raspberry pi signal contains one of the strings (“R”,”G”,”GG”,”RR”,”RG”,”GR").

  ![Event Driven Vision](/Documentation/Images/ComputerVision/EventDrivenVision.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 41: Event Driven Vision algorithm flowchart </i></p>

Some cases are visually explained below:
 - In this case two green pillars are located in the current section, so they will be both considered:

   ![Event Driven Vision](/Documentation/Images/ComputerVision/Green_Green_Case.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 42:Two green pillars case </i></p>

 - In this case two red pillars are located in the current section, so they will be both considered:

   ![Event Driven Vision](/Documentation/Images/ComputerVision/Red_Red_Case.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 43:Two red pillars case </i></p>

 - In this case only one green pillar is located in the current section:

   ![Event Driven Vision](/Documentation/Images/ComputerVision/Green_case.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 44:One green pillar in the current section case </i></p>

 - In this case one green pillars is located in the current section, and the other is located in the next section, so only the first one will be considered:

   ![Event Driven Vision](/Documentation/Images/ComputerVision/Green_Ignore_Green_case.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 45:One green pillar in the current section and another in the next section case </i></p>

- In this case, a red pillar and a green pillar are present in the current section, and one green pillar is in the next section, which won't be considered:

   ![Event Driven Vision](/Documentation/Images/ComputerVision/Red_Green_case.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 46:Red and green in pillars in current section and green pillar in the next section case </i></p>

- In this case, a red pillar exist in the current section, and one green pillar is in the next section, which won't be considered:

   ![Event Driven Vision](/Documentation/Images/ComputerVision/Red_Ignore_Green_case.jpg)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 47:Red pillar in the current section and green pillar in the next section case </i></p>

####Open Challenge Algorithm <a id="OpenChallengeAlgorithm"></a>

In this challenge, the required task is completing three laps in the least possible time, this requires constructing an algorithm that guarantees precise navigation in addition to fast and smooth movement between the sections.
Our main idea is built upon partioning the section for two areas, this approach enables us to address and solve each problem independentatly. The first area, referred to "Protected Area", contains the majority of the robot's path inside of it (Green area) as illustrated in the figure below. The second area, which is  "Unprotected Area", is the unsafe area in which the robot becomes very close to the wall as illustrated in the figure below (Red area).

![Pre-Processing flow chart](/Documentation/Images/ObstacleManagement/Area.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 48:Protected and unprotected area illustration </i></p>

#####Moving in Protected Area

In the Protected area, the robot can move safely without the risk of hitting the wall, building upon this fact, it is enough for the robot to keep moving in a straight line and parallel to the wall (no high risk of hitting the wall). PD algorithm was used to keep the robot's angle constant, the algorithm takes the reading of the gyroscope as input, and setpoint is taken after the completion of the turn. Equation below represents the formula for calculating the error and the needed angle for the servo to turn: 

$PIDOutput = K_{p(protected)} \times Error_{t} + \frac{Error_{t} - Error_{t-1}}{IntervalLength} \times k_{d(Protected)}$

$Error_{t} = GyroscopeReading - SetPoint$

However, relying only on the gyroscope sensor arises inaccurate setpoint calculation sometimes. This can be seen after the completion of the robot's turn, the robot may not turn exact 90 degrees, it can be less or more. Therefore, a correction formula is utilized to correct gyroscope's reading and calculate the correct setpoint:

$Angle = tan^{-1}(\frac{D_{S1} - D_{S2}}{D_{S1S2}})$

![ِAngle Calculation](/Documentation/Images/ObstacleManagement/AngleCal.png) <p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 49: illustration of angle calculation</i></p>


Final output can be calculated using the formula below:

$ServoAngle = Angle + PIDOutput$

Where:

- $PIDOutput$ is the calculated servo angle using PD only.
- $SetPoint$ is the reference angle (in which the robot is parallel to the wall).
- $Error_{t}$ is the difference between the current reading and $SetPoint$ in time $t$.
- $K_{p}$ is the proportional gain.
- $K_{d}$ is the derivative gain.
- $IntervalLength$ is the time period to take one measurement.
- $D_{S1}$ is the front right/left sensor reading.
- $D_{S2}$ is the rear right/left sensor reading.
- $D_{S1S2}$ is the distance between the front right/left sensor and the rear right/left sensor.
- $Angle$ is the angle between the robot and the wall.
- $ServoAngle$ is the final servo angle.

#####Moving in Unprotected Area

When entering the unprotected area, the movement is unsafe inside this area. The robot will have to immediately move away from the unprotected area until it reaches the protected area. We used P controller with a relatively high proportional gain:

$ServoAngle = K_{p(unprotected)} \times (D_{vertical} - D_{unprotected})$

$D_{vertical} = cos(Angle) \times \frac{D_{S1}+D_{S2}}{S2}$ 

where:

- $ServoAngle$ is the turning angle of servo motor.
- $K_{p(unprotected)}$ is the proportional gain in unprotected area.
- $D_{vertical}$ is the vertical distance between the robot and the wall.
- $D_{unprotected}$ is the unprotected area width.

The vertical distance was chosen to inlcude in the formula because it measures the accurate position of the robot relative to the wall, this yield more accureate results compared to the measured distance alone by one sensor or the average of the two. The distance can be calculated using simple geometry.

#####Detecting turns and direction

Once the robot detects a significant distance (>100cm), The robot identifies a turn. However, the robot can be not parallel to the wall in a way that results in wrong ultrasonic readings, as a consequence of the small this rein this case, the robot detects the front wall using front ultrasonic sensor, in case it is impossible to turn safely, the robot move backward until it reaches a point where the turn is safe.
The robot will determine the direction of movement (CW/CCW) in the first turn, this can be done by determining on which side of the robot the turn was detected. 

####Obstacle Challenge Algorithm  <a id="ObstacleChallengeAlgorithm"></a>

We mentioned previously that we have developed two algorithms, one that continuously process the scene and send information "Persistent Vision", and one that work only when it receives a signal from the arduino to start processing and send the information once "Event Driven Vision".
Based on the chosen algorithm, the robot follows a unique way to manage its movement and avoid red and green obstacles, we'll first discuss the general approach and mathematical formulas that drives the robot movements when avoiding the obstacles, then we'll discuss in depth each algorithm separately.

#####General Approach and Mathematical Formulas <a id="GeneralApproachandMathematicalFormulas"></a> 

This section presents the general approach and mathematical formulas utilized in the algorithm design for the robot's movement and obstacle avoidance. 
The primary objective of our approach is to ensure the robot moves in a manner that guarantees a safe path without collisions. 
So During the algorithm's design stage, our main motivations were focused on achieving robustness, adaptability, and fault tolerance.

#####1 - Robustness and Adaptability

 - The algorithm is designed to function effectively regardless of the robot's relative position to obstacles.
 - It takes into account various scenarios where the robot may be situated in proximity to obstacles from different angles or distances.
 - By considering relative position independence, the algorithm ensures consistent obstacle avoidance irrespective of the robot's location.

#####2 - Fault Tolerance

 - It is essential to address possible faults in the robot's movements that can affect the obstacle avoidance and ensure uninterrupted obstacle avoidance.

Those considerations led us to a mathematical formula, that calculates the needed angle to turn, we considered three cases:

#####1- Normal Case
 
- The robot receives a signal from Raspberry Pi indicating that the robot should turn to avoid the pillar, the robot will calculate the desired angle using the formula:

  ![Persistent Vision](/Documentation/Images/ObstacleManagement/DesiredAngleFormula.png)

 - D<sub>obstacle</sub>
  is the distance between the robot and the obstacle.
 - D<sub>wall</sub>
  is the distance between the robot and the wall.
 - DesiredAngle is the angle that the robot should turn to avoid colliding with the wall.
  ![Avoid](/Documentation/Images/ObstacleManagement/avoid.png)
 <p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 50: illustration of Desired angle calculation </i></p>

#####2- Safe Case

- The robot receives a signal from Raspberry Pi indicating that the robot should turn to avoid the pillar, the robot will calculate the desired angle using previous formula, if it is less than 11° degrees, this indicates that the robot is within the limits of the safe range (where it can continue its movement in a straight line without the need to turn), it sets the angle to zero, which means that the robot won't turn.

#####3- Extreme Angle Case

 - The robot receives a signal from Raspberry Pi indicating that the robot should turn to avoid the pillar, the robot will also calculate the desired angle using previous formula, if it is higher than a certain value 64° degrees, this indicates that the robot is dangerously far from the wall in a way that makes it have to turn a big angle. We set an upper limit to fix this angle, this ensures that the robot will turn safely.

After achieving the desired angle, the robot will move straight along the angle that was turned, using a PD controller to maintain a steady path until it reaches a specified distance from the wall. This distance is not the vertical distance from the wall:

 1- If the robot heading towards the outer wall, it will keep moving until the front radical sensor, which is on the wall side, read lower than a specified distance.
  
 2- If the robot heading towards the inner wall, it will keep moving until the back radical sensor, which is on the outer wall side, read higher than a specified distance.

The Arduino will send signal to the servo motor to turn the same angle, but this time in the opposite direction using Gyroscope sensor.
In both Normal case and Extreme Angle case, the used formula considers that the robot's angle is 90 degrees. However, some errors can happen that may lead to more or less than 90 degrees, we use the formula below to calculate this angle simple geometry and correct the desired angle:

  ![Persistent Vision](/Documentation/Images/ObstacleManagement/AngleCorrection.png)

It should be noted that both equations yield results in Radian, so a conversion formula is used to convert the angles to degrees:

  ![Persistent Vision](/Documentation/Images/ObstacleManagement/Conversion.png)

####Navigation Methods

The process of robot’s navigation at Straight-Froward sections controls the movements of robot after passing the pillar until it reaches a corner section. We divide these movements into three main methods:

- Follow-Right (for avoiding red pillar)
- Follow-Left (for avoiding green pillar)
- Follow-Middle only when robot detects no traffic sign (it is applicable for first section).

for every method we focused on the development of robust navigation methods to ensure safe and stable following of a robot. A fundamental requirement is the implementation of a reliable following algorithm capable of maintaining these essential criteria. While the Proportional-Derivative (PD) following algorithm effectively ensures a safe path, it keeps the robot at a calibrated setpoint (constant path) parallel to the wall, exploiting its dynamic capabilities. 
The PD algorithm enables the vehicle to maintain a zero degrees angle, thereby keeping the robot moving in a straight trajectory. The angle calculation is derived from a specified formula based on ultrasonic readings. 
Nonetheless, scenarios may arise where the robot approaches the wall in a hazardous manner, necessitating a prompt response. To handle such situations effectively,
we devided the area within the straight forward section into two distinct regions: a protected area and an unprotected area. In the protected area, the robot utilizes the PD controller to follow the specified angle. Conversely, the unprotected area mandates the robot to execute abrupt turns, facilitating a swift exit from the dangerous zone and subsequent return to the protected area for continued PD following.

####AVOID PILLARS (GENERAL)
These steps outline the process for navigating a vehicle using a gyroscope and radical sensors.
The navigation will be divided into three distinct phases:


- Phase 1: Turn a Fixed Angle using the Gyroscope

  i. The navigation process starts by receiving an ”R” or ”G” signal to the Raspberry Pi.

  ii. Use the gyroscope to turn a fixed angle in either a clockwise (CW) or counterclockwise
  (CCW) direction, depending on the scenario.

- Phase 2: Move Straight using the PD Controller

  i. Move straight along the angle that was turned, using a PD controlling to maintain a
  steady path.

  ii. Continue moving straight until the radical sensor reads a close distance. Depending on
  the situation, either the front or back radical sensor will be utilized.

- Phase 3: Turn in the Opposite Direction and Continue

  i. Turn the same angle as in phase 1, but in the opposite direction.

  ii. Continue following either the right or left path, based on the particular scenario.

####Movement In Corner Section <a id="MovementInCornerSection"></a>

These steps outline the process for navigating a vehicle in corner sections through three distinct scenarios. The following steps are considered for a counterclockwise (CCW) direction. For (CW) steps for scenarios 2 and 3 will be reversed.

#####Scenario 1: Entering the Corner from Follow-Middle

- Phase 1: Move in Follow-Middle

  i. Continue moving in Follow-Middle within the corner section until the front sensor reads lower than 30cm.

- Phase 2: Turn Backward and Calibrate

  i. Turn backward for 90 degrees CCW to face the following straight forward section, using the gyroscope.

  ii. The turn-back move is calibrated to be finished in the middle of the corner (close to the outer border and not too far from it).

- Phase 3: Move Backward and Turn

  i. Move backward (angle-Right) until the back sensor reads lower than 10cm.

  ii. Move to the following straight forward section in the middle.

#####Scenario 2: Entering the Corner from Follow-Right

- Phase 1: Move in Follow-Right

  i. Continue moving in Follow-Right within the corner section until the front sensor reads lower than 50cm.

- Phase 2: Turn Forward

  i. Turn for 90 degrees CCW to face the following straight forward section, using the gyroscope.

- Phase 3: Move to the Following Straight Forward Section

  i. Move to the following straight forward section in the middle.

#####Scenario 3: Entering the Corner from Follow-Left

- Phase 1: Move in Follow-Right

  i. Turn forward for 30 degrees CW, using the gyroscope.

- Phase 2: Move Straight

  i. Move straight following the same angle you have turned until the radical sensor reads lower than 50cm.

- Phase 3: Turn and Repeat
i. Turn the angle in phase 1 in the opposite direction.

  ii. Repeat phases 1, 2, and 3 of scenario 1 to continue through the corner section.

####Obstacle Management With Persistent Vision <a id="ObstacleManagementWithPersistentVision"></a>

These Scenarios outline the process for navigating a vehicle through three distinct scenarios using
a gyroscope and radical sensors, and detecting pillars using serial communication and image
processing:

- Phase 1: 2 pillars in the section or one pillar in the beginning of the section

  i. . If a big letter ”R” or ”G” is received, check if there is a close pillar using the back
  sensor (lower than 70cm).

  ii. If the R or G signal is sent from the Raspberry Pi for a pillar closer than 50cm, that
  means the pillar is in the beginning of the section.

  iii. Repeat phases 1 and 2 of the Avoiding Pillars navigation, but make sure to use the
  Radical Back sensors when needed.

  iv. When phase 2 is completed, turn slightly inward to the straight forward section to check
  for another pillar.

  v. If there is no pillar or a pillar of the same color, continue following either the right or left
  path.

  vi. If there is a pillar of a different color, repeat phases 1, 2, and 3 of the Avoiding Pillars
  navigation.

- Phase 2: 1 pillar in the section in the middle or the end of the section
  The checking process in the first scenario gives information about where the only pillar
  is located and if the pillar is located in the middle or the end of the section the robot will
  continue following straight until it gets “R” or “G’ from the raspberry pi, and then it bypasses
  the pillar using general steps until it gets to the corner section.

####Obstacle Management With Event Driven Vision <a id="ObstacleManagementWithEventDrivenVision"></a>

This algorithm can be divided into three steps:

  When the robot is in the proper place at start of cornern section,
   and facing the next straight forward section. The controller (Atmega) will send a signal containing either letters
 "L" or "R", which indicates the direction of the vehicle (CW or CCW). Eventually, the processor recieves the signal and starts 
 to take a capture for the faced straight forward section and identify the number of pillars in this section and their colors.
  Then it sends a signal to the controller taking the different possible cases. The vehicle deals with these cases as mentioned 
  in [General Approach and Mathematical Formulas pillars cases](#subtitle1) based on previously established color information without looking for a second pillar in the section .
**Computer vision referral**

####Comparasion Between Persistent Vision and Event Driven Vision algorithms <a id="ComparasionBetweenPersistentVisionandEventDrivenVisionalgorithms"></a>

| Algorithm | Event Driven Vision | Persistent Vision |
|-----------|:------------------:|:----------------:|
| Approach  | - Waits for a signal indicating the robot is in the first section and stops. <br>- Processes the scene using the camera only once and sends information as a signal to the Arduino controller. <br>- Robot knows exactly what to do in the current section. | - Processes the scene continuously without stopping until the three laps are completed. <br>- Constantly analyzes the image to detect and react to obstacles in real-time. <br>- After avoiding a pillar, the robot turns a small angle to check for the existence of another pillar.|
| Advantages | - Provides precise information about the current section, enabling accurate decision-making. <br>- Robot knows exactly what actions to take based on the processed image. | - Real-time processing allows the robot to react swiftly to obstacles. <br>- Continuous processing minimizes the risk of missing or misclassifying pillars. <br>- Less reliant on accurate long-distance recognition of red and green pillars. |
| Disadvantages | - Relies on a signal to indicate the start of each section, which would suffer from delays or synchronization issues. <br>- May struggle with red or green pillars isolating and distance calculation from a distance, which may lead to incorrect actions. | - Lacks prior knowledge of the current section, requiring continuous processing and decision-making. <br>- Requires the robot to turn and check for the presence of pillars, potentially affecting efficiency and speed. |
| Suitability | - Suitable when accurate knowledge of each section is crucial for decision-making. <br>- May be suitable for scenarios where pillars are reliably detectable from a close distance. | - Suitable for real-time obstacle detection and avoidance. <br>- Can handle scenarios where long-distance recognition of pillars is challenging or unreliable. |

####Efficient Play-field Pillar Data Storage
When the play-field randomization has been completed, and the pillar colors and their positions within the play-field remain constant over the course of three laps, it becomes advantageous to retain the data regarding the play-field's pillar locations and colors. The optimal approach is for the robot to capture this information during the first lap and subsequently suspend image processing, relying solely on the stored data for subsequent laps.
As our algorithm is designed to account for the positions of these pillars, it is worthwhile to store this information in a List. Each element in the list should contain comprehensive information about a particular section, as explained in detail within the code.

####Last Lap
 
The determination of the final turn direction hinges on the color of the pillar encountered during the last lap, specifically in the second-to-last lap. Two scenarios arise: if the pillar's color is green, the robot should continue the final turn in the same direction. Conversely, when the pillar is red, the robot is required to execute a reversal of the previous turn. The process of reversing the direction is undertaken within the corner section and is detailed as follows:

To transition from a counterclockwise (CCW) to a clockwise (CW) direction, the following steps are followed:

- Continue moving until the front distance sensor registers a distance of less than 75cm.

- Execute a 90-degree turn in the CCW direction.

- Move in reverse until the rear distance sensor indicates a distance less than 45cm.

- Move forward in the CCW direction until the robot reaches an angle of 30 degrees.

- Turn 80 degrees and proceed in reverse in the CCW direction.

- Continue moving in reverse until the rear distance sensor detects a distance less than 10cm.

---
###Conclusion
In conclusion, the autonomous vehicle we created for successfully solved the problems proposed in the WRO 2023 Future Engineers Category (Open Challenge - Obstacle Challenge) with the integration of mechanical and mathematical engineering principles, programming, and computer vision.  The advanced programming, real-time decision-making and computer vision algorithms enabled the vehicle to plan paths and make informed choices leading to a vehicle can navigate autonomously, aligning with the goal of connecting the world. Finally, the algorithm can improved to solve the tasks in shorter time and this project paves the way for further advancements and underscores the potential for a more connected world through innovative engineering solutions.





