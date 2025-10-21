#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Ultrasonic Sensor Driving Base Program
-----------------------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor(Port.S1)

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)

# The DriveBase is composed of two motors, with a wheel on each motor.
# The wheel_diameter and axle_track values are used to make the motors
# move at the correct speed when you give a motor command.
# The axle track is the distance between the points where the wheels
# touch the ground.
robot = DriveBase(left_motor, right_motor, wheel_diameter=50, axle_track=155)

knapp = TouchSensor(Port.S4)


# Play a sound to tell us when we are ready to start moving
ev3.speaker.beep()

# The following loop makes the robot drive forward until it detects an
# obstacle. Then it backs up and turns around. It keeps on doing this
# until you stop the program.
times_pressed=0

while True:
    if times_pressed>=2:
        break
    if knapp.pressed():
        times_pressed+=1
        ev3.speaker.say("exercise 2")
        while True:
            robot.drive(200, 0)
            # Wait until an obstacle is detected. This is done by repeatedly
            # doing nothing (waiting for 10 milliseconds) while the measured
            # distance is still greater than 300 mm.
            while obstacle_sensor.distance() > 100:
                wait(1)
                if knapp.pressed():
                    break
            if knapp.pressed():
                times_pressed+=1
                break

                # Drive backward for 300 millimeters.
            robot.straight(-100)
        
            # Turn around by 120 degrees
            robot.turn(120)
            

ev3.speaker.say("Exercise done")
