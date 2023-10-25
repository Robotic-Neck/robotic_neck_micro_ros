# robotic_neck_micro_ros
PlatformIO repository to develop the microcontroller software with the Micro-ROS framework for Robotic Neck capston project.

This repository will contain the code, libraries and other resources necessary for the successful programming and operation of the Robotic Neck system.

## Indie
1. [Description](#Description)
2. [Dependencies](#Dependencies)

## Description
El proyecto Robotic Neck focuses on the development of a robotic system that aims to expand the field of vision achievable with a single LIDAR by providing mobility in the pitch and roll axes.

This system is implemented using the Raspberry Pi Pico microcontroller to control and coordinate various functions and tasks related to the different modules within the structure's dynamics.

### Low level diagram of the electronic system
<p align="left">
  <img width="600" height="440" src="/docs/imgs/low level diagram.png">
</p>

### Electrical schematic
<p align="left">
  <img width="600" height="440" src="/docs/imgs/circuito.png">
</p>

## Dependencies
### Software
* Operating system: [Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* Robotic Frameworks:
  * [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  * [Vulcanexus (base)](https://docs.vulcanexus.org/en/humble/rst/installation/linux_binary_installation.html)
* Microcontroller Framework: [Micro-ROS](https://micro.ros.org)
* IDE: [PlatfotmIO on Visual Studio Code](https://platformio.org/install/ide?install=vscode)

#### PlatfotmIO
* [micro-ROS library](https://github.com/micro-ROS/micro_ros_platformio)

### Electronic system
#### Low level diagram of the electronic system
<p align="left">
  <img width="600" height="440" src="/docs/imgs/low level diagram.png">
</p>

#### Electrical schematic
<p align="left">
  <img width="600" height="440" src="/docs/imgs/circuito.png">
</p>

#### Hardware
* Microcontroller: [Raspberry PI pico](https://es.aliexpress.com/item/1005005492271193.html?spm=a2g0o.detail.0.0.a300BYAXBYAXSc&gps-id=pcDetailTopMoreOtherSeller&scm=1007.40000.327270.0&scm_id=1007.40000.327270.0&scm-url=1007.40000.327270.0&pvid=80bd1aaa-1118-4132-9205-e19beda6f5bd&_t=gps-id:pcDetailTopMoreOtherSeller,scm-url:1007.40000.327270.0,pvid:80bd1aaa-1118-4132-9205-e19beda6f5bd,tpp_buckets:668%232846%238109%231935&pdp_npi=4%40dis%21CLP%213505%213505.0%21%21%213.95%21%21%40210324a716916205231051502e6731%2112000033294465105%21rec%21CL%21%21A&search_p4p_id=202308091535231367361898504487695602_0)
* H-bridge: [TB6612FNG](https://es.aliexpress.com/item/32804187775.html?pdp_npi=2%40dis%21CLP%21CLP470%21CLP444%21%21%21%21%21%40210324a716916203262676043e6731%2112000034258280910%21btf&_t=pvid%3Aa1d9a860-e94b-40b5-a4a7-1f11fbd09405&afTraceInfo=32804187775__pc__pcBridgePPC__xxxxxx__1691620326&spm=a2g0o.ppclist.product.mainProduct&gatewayAdapt=glo2esp)
* Motors: [JGA12-N20 DC/Gear(50->1)/300RPM/6V](https://www.amazon.com/JGA12-N20-Motor-Speed-Encoder-3000rpm/dp/B0C2V9QTJ2?th=1)
* IMU: [GY-87/MPU6050](https://es.aliexpress.com/item/1005005002758134.html?spm=a2g0o.productlist.main.31.2ce07181negliP&algo_pvid=87703ceb-05fc-443c-8466-862c884f6110&aem_p4p_detail=2023080815300810421502957714640017224522&algo_exp_id=87703ceb-05fc-443c-8466-862c884f6110-15&pdp_npi=4%40dis%21CLP%213918%213016.0%21%21%214.43%21%21%40210318b816915338088023396e2036%2112000031310085612%21sea%21CL%210%21A&curPageLogUid=CLX3FBmFSdhz&search_p4p_id=2023080815300810421502957714640017224522_16)
* Buck converter: [LM2596 DC-DC buck converter](https://es.aliexpress.com/item/1005004354573123.html?srcSns=sns_WhatsApp&spreadType=socialShare&bizType=ProductDetail&social_params=21155296584&aff_fcid=4f77adf0b9d74611a75504fb32662668-1697222479272-08118-_m06AhcS&tt=MG&aff_fsk=_m06AhcS&aff_platform=default&sk=_m06AhcS&aff_trace_key=4f77adf0b9d74611a75504fb32662668-1697222479272-08118-_m06AhcS&shareId=21155296584&businessType=ProductDetail&platform=AE&terminal_id=5a85732ff3724ddcb8680d0a8952b024&afSmartRedirect=y)
* Limit switch: [Creality model](https://es.aliexpress.com/item/1005001834951972.html?spm=a2g0o.productlist.main.9.3894oUWRoUWROB&algo_pvid=c2cab6e0-5eae-4ec9-9755-b381f5ca6279&aem_p4p_detail=202310131148017729036743569600004645563&algo_exp_id=c2cab6e0-5eae-4ec9-9755-b381f5ca6279-4&pdp_npi=4%40dis%21CLP%211489%211344.0%21%21%211.54%21%21%402103244b16972228814054994e172b%2112000017795457109%21sea%21CL%210%21AB&curPageLogUid=cORvOKONY65a&search_p4p_id=202310131148017729036743569600004645563_5)

#### PCB
Our PCB design:
<p align="left">
  <img width="380" height="270" src="/docs/imgs/PCB.png">
</p>

## Install
Install the repository as a platformIO project:

<p align="left">
  <img width="380" height="230" src="/docs/imgs/git_clone_pio.png">
</p>

the dependencies should take a while to download and compile the dependencies.

## Documentation
TODO

## Tutorials
* [Linux course for robotic](https://app.theconstructsim.com/courses/linux-for-robotics-40/)
* [Basic C++ for robotic](https://app.theconstructsim.com/courses/59)
* [Beginner ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
* [Getting started micro-ROS](https://docs.vulcanexus.org/en/humble/rst/microros_documentation/getting_started/getting_started.html)


# Demo
Go to your platformio.ini file to set the build_src_filter:
```
build_src_filter = +<examples/pub_sub_timer/*> -<.git/> -<.svn/>
```
Now compile and upload the firmware.uf2 file to raspberry PI pico, use the one that is inside the .pio/build/pico/ folder.

Find your serial [device name]:
```
ls /dev/serial/by-id/*
```
Start micro_ros_agent:
```
ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
```
visualice msgs with ros Qt tools
```
rqt 
```
Turn down the LED low
```
ros2 topic pub /micro_ros_subcriber std_msgs/msg/Int32 data:\ 0\
```
Turn down the LED high
```
ros2 topic pub /micro_ros_subcriber std_msgs/msg/Int32 data:\ 1\
```

## Examples
All the examples have a description of how to use it in it:
* [Publisher, subcriber and timer](/src/examples/pub_sub_timer/main.cpp)
* [two encoders and motors](/src/examples/two_enc_motor/main.cpp)

## Reference
* [Micro-ROS Tutorial](https://micro.ros.org/docs/tutorials/core/overview/)
* [Vulcanexus Micro Tutorials](https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/micro_tutorials.html)
* [Reconnections and liveliness](https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/handle_reconnections/handle_reconnections.html#client-side-ping-api)

