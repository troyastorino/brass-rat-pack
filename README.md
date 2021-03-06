The code for the Brass Rat Pack rings robot.

We'll use python 2.7 for this (2.6 should be fine too). If you have python 3.x, it's probably not worth uninstalling it, just make that you don't use new python 3.x features (there aren't that many, so it also shouldn't be an issue.

In order to have your system set up to run with the robot, you must install pyserial. pyserial is used to communicate with the motors and with the Arduino. You can get it from http://pyserial.sourceforge.net/, or you can get it through a package manager:

If you have pip, you can just use that

    $ sudo pip install pyserial
    
Or with Debian/Ubuntu, you can get it through apt.

    $ sudo apt-get install python-serial

Feel free to add important info to the readme that will for setup, conceptual understanding, etc.

IMU sensor data format: (with tabs between each value)
ypr1[1]  ypr1[2]  ypr1[3]  gyro1.x  gyro1.y  gyro1.z  ypr2[1]  ypr2[2]  ypr2[3]  gyro2.x  gyro2.y  gyro2.z

Where: 
- ypr'n'.['b'] is Yaw (b=1), Pitch (b=2) and Roll (b=3)
- gyro'n'.'a' is the angular velocity about axis a
- n indicates which IMU the data is from. 

Connector pin wires
1 - 12V POWER
2 - 12V GND
3 - DYNAMIXEL DATA
4 - DYNAMIXEL N/C
5 - DYNAMIXEL GROUND
