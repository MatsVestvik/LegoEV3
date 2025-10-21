#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------
This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3
Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from math import atan, pi
import time
import random
from random import choice, randint
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import SoundFile

ev3=EV3Brick()

# Party 
last_time=time.time()
pause_numbers=0
FUN=[
    "What up world baby",
    "Mibombaclat Hilsen adrian",
    "AAAAAAAAAAAAAAAAAAA",
    "Hello World hilsen Mats"
]
def party():
    # Stop
    robot.stop()
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    # Velg en triks
    triks = randint(0,3)
    if (triks == 0): 
        # Snakking
        ev3.speaker.say(random.choice(FUN))
        
    elif (triks == 1):
        # noscope
        print("doing a 360")
        back_motor.run_target(400, 90, wait=True)
        left_motor.run(-400)  # Run at 400 deg/s
        right_motor.run(400)

        wait(2650)
        left_motor.stop()
        right_motor.stop()
        back_motor.run_target(400, 0, wait=False)


    elif (triks == 2):
        print("Trying to shake my ass")
        back_motor.run_target(400, 100, wait=True)
        back_motor.run_target(400, -100, wait=True)
        back_motor.run_target(400, 100, wait=True)
        back_motor.run_target(400, -100, wait=True)
        back_motor.run_target(400, 100, wait=True)
        back_motor.run_target(400, -100, wait=True)
        back_motor.run_target(400, 100, wait=True)
        back_motor.run_target(400, -100, wait=True)

        back_motor.run_target(400, 0, wait=True)

    elif (triks == 3):
        # Explodes
        ev3.speaker.set_volume(30)
        ev3.speaker.play_file("music/dies.mp3")
        print("Finished")
        triks-=3
    else:
        print()
    ev3.speaker.beep(frequency=2000, duration=300)
    return

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
back_motor = Motor(Port.D, gears=[8, 28])
back_motor.reset_angle(0)

L = 0.14
r = 0.02
w = 0.19
vinkel_sving=0


# Initialize the color sensor.
line_sensorL = ColorSensor(Port.S4)
line_sensorR = ColorSensor(Port.S1)
crash_sensor = UltrasonicSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = -10 # (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -100

# Volume:
ev3.speaker.set_volume(100)

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 4


# Start following the line endlessly.
print(crash_sensor.distance())
while crash_sensor.distance() > 100:


    # Calculate the deviation from the threshold.
    deviationL = line_sensorL.reflection() - threshold
    deviationR = line_sensorR.reflection() - threshold
    #print(deviation)

    current_time=time.time()
    if current_time - last_time >= 30:
        robot.stop()
        party()
        # Calculate new time
        time.sleep(2)
        pause_numbers += 1
        last_time = current_time
        continue
    if (deviationL <= 30 and deviationR > 30): # Venstre ser hvit
        #print("hvit")
        # Calculate the turn rate.
        turn_rate = -(PROPORTIONAL_GAIN * deviationL)
        DRIVE_SPEED = -20
        robot.drive(DRIVE_SPEED, turn_rate)
    elif (deviationL > 30 and deviationR <= 30): # Høyre ser hvit
        #print("hvit")
        # Calculate the turn rate.
        turn_rate = (PROPORTIONAL_GAIN * deviationR)
        DRIVE_SPEED = -20
        robot.drive(DRIVE_SPEED, turn_rate)
    elif ((deviationL > 30 and deviationR > 30) or (deviationL <= 30 and deviationR <= 30)): # Begge ser samme
        #print("svart")

        DRIVE_SPEED = -70
        robot.drive(DRIVE_SPEED, -3)
        #print(deviation)
    vL = (pi/180) * left_motor.speed()*r
    vR = (pi/180) * right_motor.speed()*r

    try:
        vinkel_sving = atan((2*L*(vR - vL)) / (w * (vR + vL))) * (180/pi)
    except:
        back_motor.run_target(400, 0, wait=False)
    else:
        back_motor.run_target(400, -vinkel_sving, wait=False)
    # You can wait for a short time or do other things in this loop.
    wait(10)
print("Stopped")
robot.stop()
left_motor.stop(Stop.BRAKE)
right_motor.stop(Stop.BRAKE)
ev3.speaker.play_file(SoundFile.FANFARE)
back_motor.run_target(200, 0, wait=True)