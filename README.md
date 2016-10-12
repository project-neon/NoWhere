# 2016 Very Small Size Robot Firmware
This firmware consists of 3 distinct codes being run on the Robot and the radio
Station.

* `Master`: Is the main firmware that controls the robot.
* `Odometry`: Communicates with the Master controller sending Mouse and IMU data
* `Station`: Communicates with the computer and sends packets to the robots.


# Parts of the `Master` puzzle

* `attitude`: Adds a thread to read the Inertial Masurement Unit constantly.
* `commander`: Handles data received via radio communication.
* `controller`: Controls the robots motor power, threads callbacks and PID constants.
* `imu`: Measures angles and accelerations from the Inertial Masurement Unit on the robot.
* `motors`: Initiates the pinMODEs, sets power and directions of the motors.
* `pid`: Configures PID control on the angular velocity of the robot.
* `robot`: Configurates the correct pinMODEs and beeps, defines the current robot's state.
* `system`: Initializes and verifies the robot's activities, such as Serial communication, LEDs and battery level.

# Looking for older software version?
We are using the same branch for all the years, and tagging them with the
year. [Click here](https://github.com/UFABC-NoBox/NoWhere/tree/vss2015)
to check out the 2015 version.

# Contributors

* [Ivan Seidel](http://github.com/ivanseidel)
* [Gabriel Bitencourt](https://github.com/gabriel-bitencs)
* [Gustavo Iha](https://github.com/gustavoiha)
