#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import random


x = Motor(Port.A)
y = Motor(Port.C)
shooter = Motor(Port.B)

X_SPEED = 100 #(max speed(1000) max range(?))
Y_SPEED = 100  #(max speed(1000) max range(800))
SHOOTER_SPEED = 500 #(max speed(?) max range(?))

startpos = 0

def shoot(num):
    current_shoter = shooter.angle()
    shooter.run_target(SHOOTER_SPEED, 90*num + current_shoter)
    

def look(x_pos, y_pos):
    x.run_target(X_SPEED, x_pos, wait=False)
    y.run_target(Y_SPEED, y_pos, wait=True)
    while not x.control.done() or not y.control.done():
        wait(10)

# Return to home position
def reset():
    y.run_target(Y_SPEED, 0, wait=False)
    x.run_target(X_SPEED, 0, wait=True)
    while not x.control.done() or not y.control.done():
        wait(10)


def testy():
    look(0,300)
    look(0,0)
    look(0,300)
    look(0,0)
    look(0,300)
    look(0,0)

def testx():
    look(150,0)
    look(-150,0)
    look(150,0)
    look(-150,0)
    look(150,0)
    look(-150,0)

def test():

    x_min, x_max = -500, 500 
    y_min, y_max = 150, 400  
    
    for _ in range(10):
        randx = random.randint(x_min, x_max)
        randy = random.randint(y_min, y_max)
        look(randx, randy)
        shoot(5)

def testspin():
    look(200,300)
    look(-300,300)
    look(400,300)
    look(-500,300)


look(230,100)
wait(1000)
shoot(2)




reset()

print("Sequence complete!")