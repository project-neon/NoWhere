# NoWhere
The Firm"Ware" of the Very Small Soccer robots, that is NoWhere


# Parts of the puzzle

* `attitude`: Adds a thread to read the Inertial Masurement Unit constantly.
* `commander`: Handles data received via radio communication.
* `controller`: Controls the robots motor power, threads callbacks and PID constants.
* `imu`: Measures angles and accelerations from the Inertial Masurement Unit on the robot.
* `motors`: Initiates the pinMODEs, sets power and directions of the motors.
* `pid`: Configures PID control on the angular velocity of the robot.
* `robot`: Configurates the correct pinMODEs and beeps, defines the current robot's state.
* `SlaveFirmware_v1`: Initializes all the classes (modules).
* `system`: Initializes and verifies the robot's activities, such as Serial communication, LEDs and battery level.


# Contributors

* [Gabriel Bitencourt](https://github.com/gabriel-bitencs)
* [Gustavo Iha](https://github.com/gustavoiha)
* [Ivan Seidel](http://github.com/ivanseidel)
