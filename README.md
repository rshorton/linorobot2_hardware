### This is a fork of linorobot2_hardware for the Elsabot PowerWheels Jeep robot base.

See https://github.com/linorobot/linorobot2_hardware for the official readme information for the linorobot2_hardware project.


# Elsabot Jeep

![Elsabot Jeep](docs/jeep_wip1.jpg)
![Elsabot Jeep](docs/jeep_wip2.jpg)


The Elsabot Jeep base:

Phase 1
* Used PowerWheels Jeep chassis
* Existing rear 12V gearbox motors modified with magnets and a Hall sensor for measuring rotational speed
* BTS7960 motor drivers
* Front steering modified to use PowerWheels steering motor with independent steering wheel shaft with encoder and motor-driver steering shaft with encoder
* GY85 IMU
* 2-18V Ryobi batteries: 1 for uC and CPU and 1 for powering motors
* 2-INA226 power monitor ICs
* Teensy 4.1 uC
* 4 port USB hub
* 433 MHz remote control relay
* Motor power relay
* RPi4
* Functionality
  * Game controller control
  * Driver (kid) control

Phase 2 (future/WIP)
* Seeed odyssey j4125 CPU
* RPLidar A1
* OAK-D camera with and pan and tilt servos
* Display panel
* USB speaker
* Seeed ReSpeaker USB Mic Array
* Various DC-to-DC converters
* 10-port USB hub
* GPS
* Functionality
  * Same as phase 1
  * Navigation control
  * Interactive functionality using Camera AI and voice control (as for previous projects)

See this WIP project for the ROS2 bring-up scripts:
https://github.com/rshorton/elsabot_jeep

Phase 2 pictures

![Elsabot Jeep](docs/jeep_phase2_pic1.jpg)
![Elsabot Jeep](docs/jeep_phase2_pic2.jpg)



