# 2018's Very Small Size Robot Firmware
This firmware consists of 3 distinct codes being run on the Robot and the radio
Station.

* `Master`: Is the main firmware that controls the robot.
* `Odometry`: Communicates with the Master controller sending Mouse and IMU data.
* `Station`: Communicates with the computer and sends packets to the robots.


# Parts of the `Master` puzzle

* `attitude`: Adds a thread to read the data comming from odometry every 1ms, this data is composed by:
  - `dx` movement in x given by the optical sensor using 400 DPI resolution,
  - `dy` movement in y given by the optical sensor using 400 DPI resolution,
  - `theta` angle in degrees from -180 to 180,
  - `inclinated` whether the robot is inclinated or not,
  - `onFloor`whether the robot is on the ground or not.
* `commander`: Adds a thread to Handle data received via radio communication whenever possible or serial communication every 50 ms, after receiving values passes them to the controller. 
* `controller`: Adds a thread that generates speed values from sensor readings and also controls the robots motor power through two PIDs one on theta speed and other on linear speed.
* `motors`: Initiates the pinMODEs,and has functions to set power and direction of the motors.
* `OdometryPacket`: Has functions to make and read odometry's communication packages, used on the attitude thread code.
* `pid`: Implements the PID controler.
* `robot`: Innitializes the robot state, data holders and features, like beeps, alarms, ids.
* `system`: Initializes robot's system, checking battery level, LEDs, for better user experience.

# Contributors

* [Ivan Seidel](http://github.com/ivanseidel)
* [João Pedro](http://github.com/joaopedrovbs)
* [Samilla Macedo](http://github.com/samillamacedo)
* [César Seiji](http://github.com/CSeijiM)
