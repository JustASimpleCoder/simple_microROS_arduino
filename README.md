# **microROS with Raspberry Pi and Arduino**
This project demonstrates how to use microROS with a Raspberry Pi and Arduino to control LEDs based on messages from a ROS2 publisher. The project explores serial communication between the Raspberry Pi and an Arduino, using microROS-like concepts without relying on the microROS library due to memory limitations on the Arduino Uno.

## **Project Overview**
### Key Components:
- Raspberry Pi running ROS2.
- Arduino Uno connected to the Raspberry Pi via serial communication.
- A simple Arduino circuit with LEDs that can be controlled using a pin number received via serial.

### **Node Structure:**
- `arduino_uno_serial` node - Handles serial communication with the Arduino Uno, sending pin numbers and receiving responses.
- `arduino_talker` node - Publishes random pin numbers (8-12) corresponding to LED control on the Arduino.
- `led_control` topic - Used for communication between the `arduino_talker` node (publisher) and the `arduino_uno_serial` node (subscriber).

## **Setup Instructions**
### Prerequisites
- A Ubuntu system is recommended for the best compatibility with ROS2.
- Install ROS2. You can follow the installation instructions here: [ROS2 Installation](https://index.ros.org/doc/ros2/Installation/).
- WSL (Windows Subsystem for Linux) can also be used if you're on a Windows machine. However, note that WSL does not have access to USB drivers by default, so you will need to set up USB access via usbipd (available here: https://github.com/dorssel/usbipd-win).

### **Folder Structure**
arduino_lister/
arduino_talker/
arduino_uno_serial/

Clone the repository to your `ros2_ws/src` directory (or create the workspace first):

mkdir -p ~/ros2_ws/src
git clone [<repository_url>](https://github.com/JustASimpleCoder/simple_microROS_arduino) ~/ros2_ws/src
The folder structure inside the src folder:

Copy code
arduino_lister/
arduino_talker/
arduino_uno_serial/
After placing the folders in src/, you need to source ROS2 in your terminal:

arduino

. install/setup.bash
Building the Packages
Navigate to your ROS2 workspace and build the packages:


cd ~/ros2_ws
colcon build --packages-select arduino_uno_serial
colcon build --packages-select arduino_talker
Running the Nodes
Open a new terminal and source ROS2:



cd ~/ros2_ws
. install/setup.bash
Run the arduino_talker node:
You should see an output like:
![image](https://github.com/user-attachments/assets/04c2a8ae-27c8-40f7-b2bd-375a3a8c5411)

ros2 run arduino_talker arduino_talker_node
In another terminal, source ROS2 again and run the arduino_uno_serial node:

bash
Copy code
cd ~/ros2_ws
. install/setup.bash
ros2 run arduino_uno_serial arduino_uno_serialized

You should see an output like and the led pins flashing:
![image](https://github.com/user-attachments/assets/92dfc785-5f2b-4ef7-85ad-fd987fc56eef)

Arduino Circuit
The Arduino Uno controls six LEDs based on the pin numbers sent via serial. This extends the basic "blink" LED functionality to support multiple LEDs.

Wiring:

Connect LEDs to pins 8, 9, 10, 11, 12 on the Arduino Uno.
Code Breakdown
Arduino Code (simplified):
The Arduino listens for pin numbers sent over serial and blinks the corresponding LED.
ROS2 Publisher Node (arduino_talker):
This node generates random pin numbers from 8 to 12 and publishes them to the led_control topic. It uses std_msgs::msg::String for communication.
ROS2 Subscriber Node (arduino_uno_serial):
This node listens for messages on the led_control topic and sends the received pin number to the Arduino via serial communication. It expects a response and logs it.
Example Publisher Code:
cpp
Copy code
// Publisher node to send random LED pin numbers
int random_pick(){
    int pins[5] = {8,9,10,11,12};
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    auto time_now = clock.now();
    int index = static_cast<int>(time_now.seconds()) % 5; // pick random pin
    return pins[index];
}
Example Subscriber Code:
python
Copy code
# Subscriber node to handle serial communication with Arduino
serial_port.write(f'{pin}\n'.encode())  # Send pin number
response = self.serial_port.read()  # Read Arduino response
Troubleshooting
Arduino Not Responding: Ensure the correct port is specified in the ROS2 code (/dev/ttyUSB0). You can check the connected devices with ls /dev/tty*.

WSL USB Access: If you're using WSL, remember that WSL does not have access to USB devices by default. Use usbipd to share USB devices between Windows and WSL.

Contributions
Feel free to fork the project and submit pull requests. If you find any bugs or issues, please open an issue in the GitHub repository.

License
This project is licensed under the MIT License - see the LICENSE file for details.

This README should provide clear instructions for setting up and running the project, as well as details on the code and how to troubleshoot any issues.

You should see an output like:
![image](https://github.com/user-attachments/assets/04c2a8ae-27c8-40f7-b2bd-375a3a8c5411)


Open another terminal, navigate to ros2_ws and source ros and launch the packages
cd ~/ros2_ws
. install/setup.bash
ros2 run arduino_talker arduino_talker_node

You should see an output like and the led pins flashing:
![image](https://github.com/user-attachments/assets/92dfc785-5f2b-4ef7-85ad-fd987fc56eef)


