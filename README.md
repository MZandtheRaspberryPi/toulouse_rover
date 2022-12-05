# auto_rover
this was a fun project where I made a wheeled ros based robot from scratch, including hardware, design, and substantial software for kinematics, control, and odometry (though I used existing SLAM and path planning packages).
![frontal_pic](demo_assets/frontal_pic.png)


this robot has support for differential drive (with 2 driven wheels), skid steering (with 4 driven wheels), and omni wheels.

This repo is structured as a catkin workspace, so you should just clone it with submodules, and use catkin to build the workspace.

## Demo GIFs

## Software

![node_graph](demo_assets/node_graph.PNG)  
Not pictured above is the lidar or the serial communicator node with the arduino.

You can run tests with:  
```
catkin_make run_tests toulouse_rover
```

I wrote software to do kinematics for the robot and calculate which speed wheels should turn out [WheelSpeedCalculator](src/toulouse_rover/include/toulouse_rover/wheel_speed_calculator.h) .  

I also wrote software to control the wheel speeds given encoder input, or to control the wheel speeds using a linear model to scale input from max to minimum speed [WheelSpeedController](src/toulouse_rover/include/toulouse_rover/wheel_speed_controller.h)).

I also wrote software to calculate robot position given the wheel rotation speeds, for example from encoder inputs [OdomCalculator](src/toulouse_rover/include/toulouse_rover/odom_calculator.h). I didnt end up using this as my encoders only had 20 ticks per revolution and I found it challenging to get the time step down enough to where I could rely on the output position.

The WheelSpeedCalculator listens to command velocities and then calculates what speeds wheels should turn. It then sends this to the WheelSpeedController, which calculates pwm speeds and sends this to an arduino that interfaces with the motor driver. The arduino sends the encoder data back, and this gets passed to the OdomCalculator.

I dont use the values published from OdomCalculator, instead I use Hector SLAM to publish odometry and map the room, and I use move_base to do path planning based on that. I publish command positions using rviz, and move_base publishes command velocities to achieve the goal pose.

You can find the ros package for toulouse [here](src/toulouse_rover/).

## Hardware

![wheel_hardware](demo_assets/20221205_205958.jpg)  
![upper_hardware](demo_assets/20221205_210029.jpg)


The basic hardware is all from Adafruit. The motors are yellow plastic TT Motors [here](https://www.adafruit.com/product/3777) and the motor drivers are dev boards based on the DRV8833 motor driver [here](https://www.adafruit.com/product/3297). This is all powered with a 2s LIPO at 7.4 volts, with two step down converters, one to the pi, one to the motors. The encoders are plastic encoder wheels with 20 ticks per rotation. A T slot IR interupt sensor was used to measure encoder ticks.

The brains is a Raspberry PI 4 with 4gb RAM, running ubuntu 20.04.

An Arduino Nano is used to measure the interupts from the IR sensor, as well as to send PWM to the motor driver boards. It talks to the Raspberry PI via Serial.

## Design
I wanted to build a robot with wheels, as I hadnt worked with them a lot. From there I did some sketches, and it evolved into a turtle robot with a lidar on its shell, with stacked layers inside letting me mount different electronics:
![original_drawing](demo_assets/20210703_170028.jpg)

I tried to figure out what I would need on each layer, and what components I wanted to use for electronics. From here I started doing sketches in fusion 360...

I eventually had a full design with a turtle:
![full_turtle_cad](demo_assets/turtle_cad.PNG).

And all the electronics were hidden behind the shell:
![inner_turtle_cad](demo_assets/turtle_inner_cad.PNG).

You can find cad files [here](cad).
