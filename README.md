### This is a fork of linorobot2_hardware for the Elsabot PowerWheels Jeep robot base.

See https://github.com/linorobot/linorobot2_hardware for the official readme information for the linorobot2_hardware project.


# Elsabot Jeep

![Elsabot Jeep](docs/jeep_wip1.jpg)
![Elsabot Jeep](docs/jeep_wip2.jpg)


The Elsabot Jeep base:

* Used PowerWheels Jeep chassis
* Existing rear 12V gearbox motors modified with magnets and a Hall sensor for measuring rotational speed
* BTS7960 motor drivers
* Front steering modified to use PowerWheels steering motor with independent steering wheel shaft with encoder and motor-driver steering shaft with encoder
* GY85 IMU
* 2-18V Ryobi batteries: 1 for uC and CPU and 1 for powering motors
* Teensy 4.1 uC
* Seeed odyssey j4125 CPU
* RPLidar A1
* OAK-D camera with and pan and tilt servos
* Display panel
* USB speaker
* Seeed ReSpeaker USB Mic Array
* Various DC-to-DC converters
* 10-port USB hub
* 433 MHz remote control relay
* Motor power relay

TBD:
See this project for the ROS2 bring-up scripts:
https://github.com/rshorton/elsabot_4wd

