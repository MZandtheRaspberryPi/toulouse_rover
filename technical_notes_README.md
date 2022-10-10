# auto_rover

a repository for a Pi4 based autonomous rover running ROS

## TODO

do documentation
figure out why when we turn robot real time lidar data stops aligning wtih the map
figure out why we cant autonomously navigate, robot just get stuck (probs above issue)

## Structure

A node that listens to cmd vel and calculates each wheels needed speed according to kinematics model and wheel configuration, and publishes it in radians per second.

One node that is listening to command wheels speeds and wheel encoder counts and using pid controllers or linear approximation to calculate the pwm to send to motor driver.

One node that is listening to wheel encoder counts and takes the wheel configuration, and calculates odometry.

Ros Navigation Stack. Ydlidar.

## Setup the Pi

There are some gotcha's in setting up Ubuntu 20.04 on the Pi4, namely trying to do it headless and dealing w/ the WiFi woes. See the section on this github and set it up accordingly. <https://github.com/MZandtheRaspberryPi/pi_headless_setup#setting-up-ubuntu> .

From there install ROS Noetic according to the ROS wiki: <http://wiki.ros.org/noetic/Installation/Ubuntu> . I would install ROS base onto the PI, and add the sourcing of setup.bash into your .bashrc file so that each time you load a terminal you can use ROS Noetic.  

Install wiringpi, which we will use to control GPIOs. If on a Pi4, you'll need to install from a fork of the original library as the original library is no longer maintained.

wiringpi library has since been stopped being mainted by original author, fork is here: <https://github.com/WiringPi/WiringPi>
see install directions here: <https://github.com/WiringPi/WiringPi/blob/master/INSTALL>  

To check pinouts:

```
cd ~/WiringPi
gpio readall
```

I'm still figuring out how to use it without sudo, for the individual user. When it was the original library the below was applicable:
To access GPIO pins as a normal user, follow this stack overflow: <https://askubuntu.com/questions/1230947/gpio-for-raspberry-pi-gpio-group>
Install an RPi library and add ubuntu to dailout group then reboot.

Eventually I moved away from wiringpi and ran all of the real time stuff on an arduino nano. I did have some weird bugs with udev rules I had setup for a lidar, to make a symlink to the serial port with a name. I had to remove this. Else I got these weird errors where my arduino would reset and not be found anymore. I eventually realized i start sending data without waiting for the bus to be up: <https://forums.raspberrypi.com/viewtopic.php?t=331475>  adding a `while(!Serial.available())` loop helped.

plug it and replug it, and it doesnt come back...

```
dmesg

[   52.647330] usb 1-1.4: USB disconnect, device number 5
[   52.649214] ftdi_sio ttyUSB1: FTDI USB Serial Device converter now disconnected from ttyUSB1
[   52.649313] ftdi_sio 1-1.4:1.0: device disconnected
[   55.685520] usb 1-1.3: new full-speed USB device number 6 using xhci_hcd
[   55.766008] usb 1-1.3: Device not responding to setup address.
[   55.973750] usb 1-1.3: Device not responding to setup address.
[   56.181530] usb 1-1.3: device not accepting address 6, error -71
[   56.265534] usb 1-1.3: new full-speed USB device number 7 using xhci_hcd
[   56.345808] usb 1-1.3: device descriptor read/64, error -32
[   56.537736] usb 1-1.3: device descriptor read/64, error -32
[   56.650061] usb 1-1-port3: attempt power cycle
[   57.253535] usb 1-1.3: new full-speed USB device number 8 using xhci_hcd
[   57.253798] usb 1-1.3: Device not responding to setup address.
[   57.461758] usb 1-1.3: Device not responding to setup address.
[   57.669535] usb 1-1.3: device not accepting address 8, error -71
[   57.753509] usb 1-1.3: new full-speed USB device number 9 using xhci_hcd
[   57.753771] usb 1-1.3: Device not responding to setup address.
[   57.961807] usb 1-1.3: Device not responding to setup address.
[   58.169537] usb 1-1.3: device not accepting address 9, error -71
[   58.176208] usb 1-1-port3: unable to enumerate USB device
```

and

```
ftdi_sio ttyUSB1: usb_serial_generic_read_bulk_callback - urb stopped: -32
```

In terms of controlling DC motors compile and run:

```
g++ -g test_motors.cpp -o test_motors -lwiringPi
sudo ./test_motor
```

For a refresher on gpio pins, navigate to where you saved WiringPi
ubuntu@ubuntu:~/WiringPi/gpio$ sudo ./gpio readall

For some thoughts on controlling many PWM from a raspberry pi, see the below. For real time operation, folks recommend a PWM driver board and communicating w/ that board via I2C.
<https://raspberrypi.stackexchange.com/questions/298/can-i-use-the-gpio-for-pulse-width-modulation-pwm> link on suggestions about control

## Wiring

For the voltage divider to get 3.3v power: <https://randomnerdtutorials.com/how-to-level-shift-5v-to-3-3v/>  
Voltage divider to go from 5v to 3.3:
Vout = Vin * R1 / (R1 + R2)
One solution is R1 = 1000 ohm, R2 = 2000 ohms

For the pullup resistor for listening to Optical Encoders: <https://learn.sparkfun.com/tutorials/pull-up-resistors/all?print=1>  

For monitoring the lipo voltage:
<https://www.instructables.com/1S-6S-Battery-Voltage-Monitor-ROS/>  
Where we use a voltage divider, but tailored to get half of total voltage for our 2s Lipo.

## uploading to arduino

we will use arduino cli for this <https://github.com/arduino/arduino-cli>.

```
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
```

<https://arduino.github.io/arduino-cli/latest/getting-started/> for intro commands
arduino:avr:nano is fqbn string for nano. can use core install arduino:avr.

add ros to 3rd party lib:
~/.arduino15$ nano arduino-cli.yaml

install rosserial arduino
<http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup>

on newest arduino lib ros
<https://answers.ros.org/question/361930/rosserial-arduino-compilation-error-no-cstring/>  

for rosserial commands: <http://wiki.ros.org/rosserial_python>  
sudo apt-get install ros-noetic-rosserial  
rostopic echo /battery_state
screen -dmS batt rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0_baud:=57600

Batter voltage monitor: <https://www.instructables.com/1S-6S-Battery-Voltage-Monitor-ROS/>
Bug for batt voltage: <https://bugzilla.kernel.org/show_bug.cgi?id=46201>

## For the motors

To build catkin package, make sure you have build tools installed.  
sudo apt-get update && sudo apt-get install build-essential

Init submodules

```
git submodule init
git submodule update
```

```
cd ~/toulouse_rover
catkin_make
```

<https://gitlab.com/bradanlane/ros-i2cpwmboard.git>
git submodule add <https://gitlab.com/bradanlane/ros-i2cpwmboard.git> src/ros-i2cpwmboard

sudo apt-get install libi2c-dev  

sudo usermod -a -G i2c ubuntu  
sudo usermod -a -G dialout ubuntu  
sudo usermod -a -G tty $USER  

Suggestion on gpio stuff.
chown -R root:dialout /sys/class/gpio && chmod -R 770 /sys/class/gpio;
<https://answers.ros.org/question/216441/raspberry-pi-2-ros-and-gpio-access/>

guide on how to include WiringPi (namely add -lwiringPi to executable in CMakeLists and also add in some stuff to check if on windows and such).
<https://roboticsbackend.com/use-and-compile-wiringpi-with-ros-on-raspberry-pi/>

Running the motors

```
rosrun i2cpwm_board i2cpwm_board
```

On checking if on a pi:
<https://www.raspberrypi.org/forums/viewtopic.php?t=20811>

resources i2cpwm board
<https://www.youtube.com/watch?v=iLiI_IRedhI>  
<http://bradanlane.gitlab.io/ros-i2cpwmboard/>  
<https://github.com/mike4192/spotMicro/blob/master/spot_micro_motion_cmd/src/spot_micro_motion_cmd.cpp>  
<https://github.com/tizianofiorenzani/ros_tutorials/blob/master/donkey_car/src/low_level_control.py>  

To start:
source devel/setup.bash
screen -dmS core roscore
screen -dmS batt rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0_baud:=57600
screen -dmS i2c rosrun i2cpwm_board i2cpwm_board

## Kinematics for Mecanum Wheeled Robots

<https://howtomechatronics.com/projects/arduino-mecanum-wheels-robot/>  

<http://robotsforroboticists.com/drive-kinematics/>

## Kinematics for Dif Drive Robots

<https://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf>  
To solve for odom, take arc length left wheels went, arc length right wheels went, and the arc length robot drove is equal to the average of them. Can solve for that analytically too.
<https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/> also as well as <https://robohub.org/drive-kinematics-skid-steer-and-mecanum-ros-twist-included/>

<https://robotics.stackexchange.com/questions/18048/inverse-kinematics-for-differential-robot-knowing-linear-and-angular-velocities>
helpful for units

```
sudo apt-get install ros-noetic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6 _speed:=0.1 _turn:=0.2

```

We use these wheels, 48mm diameter: <https://www.adafruit.com/product/4679>

130mm wheel seperation length

92mm wheel seperation width  

We use these optical encoder wheels: <https://www.adafruit.com/product/3782>  with 20 slots all the way round. So 20 slots is one wheel rotation.

pre-node shutdown: <https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/>  

using ROS PID
sudo apt-get install ros-noetic-pid  

sudo apt-get install ros-noetic-rqt
sudo apt-get install ros-noetic-rqt-common-plugins

rostopic pub -1 /left_front_wheel/state std_msgs/Float64 2
rostopic echo left_front_wheel/control_effort

rostopic pub -1 /left_front_wheel/setpoint std_msgs/Float64 6

for pifi:  
<https://packages.ubiquityrobotics.com/>  add to packages
<https://github.com/rohbotics/pifi>  
<https://github.com/IntelRealSense/librealsense/issues/9506>  to add key

on toulouse_robot:  
export ROS_IP=10.42.0.1

on other PC
export ROS_MASTER_URI=<http://10.42.0.1:11311>  

I was using a pi4 on the robot running ros, then a windows laptop running an ubuntu virtual machine to visualize stuff with rviz. I had a tough time getting the two to talk to each other, virtual machine and PI4. What ended up working was setting ROS_IP env variables in the .bashrc file for both (get ip from hostname -I for example), and then in the virtual machine settings under network, setting bridged, so it gets its own ip address.

Ideas for next time:  
Use arduino or 3.3v board for interupts from encoders, battery voltage measurement, and pwm signals to motor drivers.
Serial could be interface to arduino to command it.

Internet on Pi:
<https://raspberrypi.stackexchange.com/questions/109425/ubuntu-server-18-wifi-hotspot-setup>

ROSDEP

```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
cd ~/toulouse_rover
rosdep install --from-paths src --ignore-src -r -y

```

Things to do differently
Encoders to real time controller like arduino using interupts, serial to pi
Motor Driver PWM from Arduino as well if possible
Higher resolution encoders (20 ticks per rotation is tough to do PID with)
Make cable connectors better. Currently dupont, but self made and some of them dont plug in too well and hold.

install lidar packages. had issues with sdk for most recent driver version (<https://github.com/YDLIDAR/ydlidar_ros_driver>).
<https://github.com/YDLIDAR/ydlidar_ros>

## Making a map

<http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData>

for gmapping:  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" args="scan:=scan"/>

issue with power of ydlidar. Takes about an amp on spin up and my pi 4 with its power system cant provide from usb. results in other stuff in the usb ports losing power. should have seperate system for this ydlidar.

# Formating and Linting

We use google style for the clang-format, and default clang-tidy settings.

```
clang-format -style=google -dump-config > .clang-format
clang-format --style=file -i src\toulouse_rover\include\toulouse_rover\odom_calc.h
```

for clang tidy, need to know compile flags. so when building, generate .json

```
catkin_make toulouse_rover -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

clang-tidy -p ./build --format-style="file" src/toulouse_rover/src/wheels.cpp
```

# Testing

```
catkin_make run_tests
```
