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

X_SPEED = 1000 #(max speed(1000) max range(300++))
Y_SPEED = 1000  #(max speed(1000) max range(800--))
SHOOTER_SPEED = 500 #(max speed(1000) max range(20?))

# functions
def shoot(num):
    current_shoter = shooter.angle()
    shooter.run_target(SHOOTER_SPEED, 90*num + current_shoter)
    

def look(x_pos, y_pos):
    x.run_target(X_SPEED, x_pos, wait=False)
    y.run_target(Y_SPEED, y_pos, wait=True)
    while not x.control.done() or not y.control.done():
        wait(10)

#reset but lifts first to thow empty magasines for large capacity
def reset():
    y.run_target(Y_SPEED, 0, wait=False)
    x.run_target(X_SPEED, 0, wait=True)
    while not x.control.done() or not y.control.done():
        wait(10)


# different tests use test() for overall test
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
    y_min, y_max = 50, 400  
    
    for _ in range(10):
        randx = random.randint(x_min, x_max)
        randy = random.randint(y_min, y_max)
        look(randx, randy)
        shoot(2)

def testspin():
    look(200,300)
    look(-300,300)
    look(400,300)
    look(-500,300)


test()


# always include reset for simplicity!!!!
reset()

print("Sequence complete!")