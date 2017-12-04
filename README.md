# Layout Robot
Senior Design Fall 2017 Team DM6


## Code Explanation
### Raspberry Pi

The raspberrypi is configured to use i2c and host its own wireless access point called “pi-3-ap.” The password for the wifi network is “raspberry.” The raspberrypi has a static IP, but it is easier to access the raspberrypi using its hostname “roomba.local”. The login credentials for the raspberrypi are username “pi” with password “raspberry”. Example ssh login: “ssh pi@roomba.local” with password “raspberry”  

### Software

The code for the robot can be found on the provided flash drive. It runs on the onboard raspberrypi using python3. “src” contains the main code needed to run the demo. “Experiments” contains various tests and tools. “externals” contains obscure libraries that would need to be installed on a new system. All other dependencies can be installed via pip or apt-get.

- App.py: The main code. Launching this software (using sudo) will run the demo. It will launch a webserver that can be accessed via the browser at “roomba.local”
- Drive.py: Test main code. This will run a hard coded version of app.py . The waypoints are hard code and this does not launch a webserver.
- Pos_filter.py: This file contains a class which manages all the sensor fusion. It aligns coordinate systems and fuses different sensor systems. This module is designed to run at 60 hz.
- Pos_control.py: This file contains a class that defines the control logic and scheme for getting the robot in position. It is designed to be run at 60 hz.
- Hedgehog.py: This file provides a way to interface to the sonar beacons over serial. This code runs in it’s own thread which gets launched when you instantiate a hedgehog object.
- Open_interface.py: This file is a partial implementation of the icreate open interface. It allows control on the roomba’s motors and allows us to wheel encoders. It can be easily expanded to provide more functionality.
- Homography.py: This file contains a module used in aligning the beacon and encoder coordinated systems. It’s solves the homographic transformation between two lists of coordinates which are not aligned.
- Calibrate_bno.py: This file runs the calibration routine for the IMU. It will run until all systems read a calibration value of 3. It will print out the calibration coefficients which can be copied and pasted into app.py or drive.py under the variable name “bno_cal”. The calibration should be run if the mechanical configuration of the robot changes. See the file for more info on how to complete the calibration.
- Index.html: Contains the HTML webpage which acts as our user interface into the system. The user can draw floor plans, place the roomba’s origin point, and export the waypoint csv to the roomba.
- Map.js: The file makes the HTML page interactive and handle the connection to the raspberrypi using websockets.
- Websockets.js: A library which allows for bidirectional string based communication between the raspberrypi and the user’s web browser.
- P5.min.js: The Processing library which allows for simple browser based drawing.

### Tools
Included on the flashdrive is a tool (marvelmind) for setting up and viewing the beacon system.
