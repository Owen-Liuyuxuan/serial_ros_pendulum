# serial_ros_pendulum
I am too boring to start working on this comprehensive mini-project. It include parts of serial communication, ROS and inverted pendulum control. 

## 1. Introduction
This is a microproject implement something interesting but "useless". I am actually first moltivated during my learning of the connecting microprocessor with ROS through serial ports. The task can be decomposed in three parts:

    1. Simulate a controllable inverted pendulum in robot operating system (ROS).
    
    2. Develop and verify the algorithm to balance the pendulum in Matlab.
    
    3. Program a microprocessor (STM32F407) which receives the state of the pendulum from ROS, calculates and publishes the control signal to ROS. The pendulum simulated in ROS should be balanced by the microprocessor.
    
The project is actually developed on three computing machines., i.e. two computers installed with Linux and Windows repectively and a microprocessor. This is a comprehensive project to test the ability to tackle with multiple platorms. It is not that user-friendly but should be a good reference source when we encounter problems in ROS, control theory and serial communication in the future.

## 2. File Structure

   Folder "Matlab". It contains the simulink model of the inverted pendulum. All of the model and the control inputs are hardcoded in one user-defined function. The model we adopted in simulation is a nonlinear model. The zero-order hold can be neglected.

Folder "ros". It contains three ros package.
>  1. inverted_pendulum: Define msg file to convey the pendulum status, define a launch file to launch all three nodes and define a simulation node that subscibe to control input and publish pendulum status.
>  2. visualization: Contain a python script to visualize the simulation. The node will subscribe to the published pendulum status. 
>  3. try_serial: Define a ros node that subscribe to pendulum status, publish control input, communicate with the microprocessor with serial port.
   
Folder "Stm32F4": Contain a .rar file which contains a Keil5 project used to program the microprocessor. The microprocessor should be communicate with the Linux machine through serial port and calculate the controller input.

## 3. Some Dependencies and Important/Interesting Details

   1. Matlab Files depends on simulink but not other toolboxes including the control or modelling toolboxes. Everything is written in user-defined function.

   2. inverted_pendulum node depends on boost::numeric::odeint to perform simulation.

   3. visualization node depends on matpltlib in Python 2.7.

   4. try_serial node depends on boost::bind and boost::asio. This project utilize boost::asio instead of those native ros_control packages. 

   5. try_serial node needs to be connected to the correct port, which is defined in try_serial.cpp line 25.

   6. STM32F4 is developed with Keil5, ST-Link. The microprocessor is the same as [this](https://detail.tmall.com/item.htmspm=a230r.1.14.6.2ff226d9QLrN9y&id=524181346955&cm_id=140105335569ed55e27b&abbucket=19) Taobao link.

   7. Floating numbers are transmitted through serial port in its original form (4 bytes for each number). It is worth noting that, interestingly, programs to "transform" float type to unsigned char type are different in ROS and microcomputer. During the development, the program succeeds in ROS failed in microcomputer and I havn't figured out the reason.
