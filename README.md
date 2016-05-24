# ev3
Driver for the LEGO MINDSTORMS EV3. Making use of the ev3dev kernel (http://www.ev3dev.org) and ROS it makes it easy to program some of the functionalities of this block in a Linux environment.

# 1. INSTALATION

## 1.1 SERVER
In order to install the server part of the driver. The contents of this folder have to be copied into the block. To do so, please make sure to have followed the setup steps in http://www.ev3dev.org/ and you have a working connection with the block. When the setup is ready type the following in a terminal:

scp -r ev3pkg_ev3/ <ev3_user_name>@<ev3_ip_address>:
i.e: scp -r ev3pkg_ev3/ev3pkg aratz@10.42.0.3:

Once this has been done run the script ev3_install in the ev3pkg folder in order to activate the driver on startup. This scrip takes a parameter which is the the absolute path of the driver itself.

sudo ./ev3_install /aratz/ev3pkg && reboot


## 1.2 CLIENT
This driver is a Catkin ROS node. Therefore it is installed like anyother ROS package from the source.

# 2. Execution
Once the ev3pkg package has been installed a call to its ROS launcher is enough for its execution:

roslaunch ev3pkg ev3pkg.launch
