#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
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

knapp = TouchSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

threshold = 14
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -125 # Working speeds (in pair):  -150    -200    -225
TURN_SPEED = -75 #-50 #-25   
PROPORTIONAL_GAIN = 3
compare = 10

# cross_diagonal calibrations:
back_duration = 0.2
straight_duration = 0.7
time_adjust = 3

def cross_diagonal():
    # ---------------------------
    # NB: Krever nærmere sensorer for at den fungerer. (I vanlig banen)
    # ---------------------------

    # Sucsessful Calibrations
    # (Drive_speed, turn_speed, Distance_backwards, time to drive ahead)
    #   (-250, -20, 125, 0.7)
    #   (-225, -15, 125, 0.7) Workers with wider and narrower sensors
    #   (-150, -5, 50, 1)

    # Testing:
    #   (-250, -25, 125, 0.7)
    #   (-250, -20, 30, 0.7)

    # Beep to signal
    wait(10) 
    ev3.speaker.beep()
    # Force stop and reset backwheel
    robot.stop()
    back_motor.run_target(700, 0, wait=True)

    # Drive backwards
    start_time = time.time()
    while ((time.time() - start_time) < back_duration): 
        robot.drive(800, 0)

        #robot.straight(distance_back) # Higher speeds demand greater backwards distance?
    # Time-variable, used in while:
    start_time = time.time()

    # Adjust robot so it goes straight
    while ((time.time() - start_time) < time_adjust): # Does this for 2 seconds
        deviationL = line_sensorL.reflection() - threshold
        deviationR = line_sensorR.reflection() - threshold
        back_motor.run_target(700, 90, wait=False) # Sideways backwheel

        if (deviationL <= compare and deviationR > compare): # Venstre ser hvit
            #print("hvit")
            # Calculate the turn rate.
            turn_rate = -((abs(deviationL)+abs(deviationR)))/10
            print(turn_rate)
            robot.drive(-1, turn_rate)
            wait(10)


        elif (deviationL > compare and deviationR <= compare): # Høyre ser hvit
            #print("hvit")
            # Calculate the turn rate.
            turn_rate = ((abs(deviationR)+abs(deviationL)))/10
            print(turn_rate)
            robot.drive(-1, turn_rate)
            wait(10)

        else:
            turn_rate = 0
            wait(10)

    
    # Reset backmotor and hold
    back_motor.run_target(700, 0, wait=True)
    back_motor.hold()

    # Drive and reset backmotor
    start_time = time.time()
    while ((time.time() - start_time) < straight_duration):
        robot.drive(-600, 0)
        # Different distanses need more time?:


    #robot.straight(-200) 

    back_motor.run_target(700, 0, wait=True)



    # Drive logic 2
    #robot.drive(DRIVE_SPEED, 0)
    #robot.drive(DRIVE_SPEED, 0)
    #wait(300)

    #back_motor.run_target(200, 0, wait=True)

while not knapp.pressed():
    # Calculate the deviation from the threshold.
    deviationL = line_sensorL.reflection() - threshold
    deviationR = line_sensorR.reflection() - threshold
    print(deviationL, deviationR)
    if (deviationL <= compare  and deviationR <= compare ):
       cross_diagonal()

    elif (deviationL > compare+20 and deviationR > compare +20):
        robot.drive(-4*DRIVE_SPEED, 0)
        wait(20)
    
    elif (deviationL > compare and deviationR > compare): # Begge ser samme
        #print("svart")

        robot.drive(DRIVE_SPEED, 0)
        #print(deviation)
    elif (deviationL <= compare and deviationR > compare): # Venstre ser hvit
        #print("hvit")
        # Calculate the turn rate.
        turn_rate = -(PROPORTIONAL_GAIN * abs(deviationL)+abs(deviationR))

        robot.drive(TURN_SPEED, turn_rate)
        robot.drive(TURN_SPEED, turn_rate)
        robot.drive(TURN_SPEED, turn_rate)
        robot.drive(TURN_SPEED, turn_rate)

    elif (deviationL > compare and deviationR <= compare): # Høyre ser hvit
        #print("hvit")
        # Calculate the turn rate.
        turn_rate = (PROPORTIONAL_GAIN * abs(deviationR)+abs(deviationL))
       
        robot.drive(TURN_SPEED, turn_rate)
        robot.drive(TURN_SPEED, turn_rate)
        robot.drive(TURN_SPEED, turn_rate)
        robot.drive(TURN_SPEED, turn_rate)
    vL = (pi/180) * left_motor.speed()*r
    vR = (pi/180) * right_motor.speed()*r

    try:
        vinkel_sving = atan((2*L*(vR - vL)) / (w * (vR + vL))) * (180/pi)
    except:
        back_motor.run_target(700, 0, wait=False) # Before: False
    else:
        back_motor.run_target(700, -vinkel_sving, wait=False) # Before: False

    # Set the drive base speed and turn rate.

    # You can wait for a short time or do other things in this loop.
    wait(10)

robot.stop()
left_motor.stop(Stop.BRAKE)
right_motor.stop(Stop.BRAKE)
back_motor.run_target(200, 0, wait=True)