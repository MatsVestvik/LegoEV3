#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.hubs import EV3Brick


import random
import math

x = Motor(Port.A, gears=[24, 56])
y = Motor(Port.C, gears=[8, 40])
shooter_left = Motor(Port.B)
shooter_right = Motor(Port.D)


X_SPEED = 100
Y_SPEED = 100
SHOOTER_SPEED = 1000

#variables to edjust as we do testing
# initial speed m/s
v = 9

#variables to adjust as the robot is completed
#starting height
sh = 19

# functions
def shoot(num):
    current_shoter = shooter_left.angle()
    shooter_left.run_target(SHOOTER_SPEED, 180*num + current_shoter,wait=False)
    shooter_right.run_target(SHOOTER_SPEED, 180*num + current_shoter,wait=True)
    
def run_target_better(motor, target_angle):
    limit=720
    current = motor.angle()
    current_mod = current % 360
    target_mod = target_angle % 360

    diff = (target_mod - current_mod + 180) % 360 - 180

    # Foreslått nytt mål (uten grenser)
    proposed_target = current + diff

    # --- Begrensning: må være innenfor [-limit, limit] ---
    if proposed_target > limit:
        proposed_target -= 360
    elif proposed_target < -limit:
        proposed_target += 360

    actual_diff = proposed_target - current
    if abs(actual_diff) > 180:
        if (current > limit - 180 and actual_diff > 0) or (current < -limit + 180 and actual_diff < 0):
            pass
        else:
            proposed_target = current + (180 if actual_diff > 0 else -180)

    motor.run_target(X_SPEED, proposed_target, wait=False)



def look(x_angle, y_angle):
    run_target_better(x, x_angle)
    y.run_target(Y_SPEED, y_angle, wait=False)
    a=i
    if "lob" in a:
        a.remove("lob")
    elif "stab" in a:
        a.remove("stab")
    
    if targets.index(i)!=0:
        b=targets[targets.index(i)-1]
        if "lob" in b:
            a.remove("lob")
        elif "stab" in b:
            a.remove("stab")
    else:
        b=[0.01,0.01,0.01]
    print(a,b)
    cos_theta = sum(x*y for x, y in zip(b, a)) / (math.sqrt(sum(x*x for x in b)) * math.sqrt(sum(y*y for y in a)))
    cos_theta = max(-1, min(1, cos_theta))
    wait(240 * math.acos(cos_theta))

#reset but lifts first to thow empty magasines for large capacity
def reset():
    y.run_target(Y_SPEED, 0, wait=False)
    x.run_target(X_SPEED, correct_angle(0), wait=False)
    while not x.control.done() or not y.control.done():
        wait(10)

def correct_angle(angle):
    diff = (angle - x.angle() + 180) % 360 - 180
    print("diff",diff)
    return angle + (diff/abs(diff))*4

def aim(x_coord,y_coord,z_coord,mode="stab"):
    

    x_angle=math.degrees(math.atan2(y_coord, x_coord))
    if math.sqrt(x_coord**2 + y_coord**2)>150:
        y_angles = calculate_angle(math.sqrt(x_coord**2 + y_coord**2), z_coord)
        print(y_angles)
        if not y_angles:
            print("Ingen løsning for denne distansen.")
            return

        # velg vinkel basert på modus
        if mode == "lob":
            y_angle = y_angles[-1]   # høy bane
        else:
            y_angle = y_angles[0]    # lav bane
    else:
        y_angle=math.degrees(math.atan2(z_coord, math.sqrt(x_coord**2 + y_coord**2)))

    
    look(correct_angle(x_angle), y_angle)

def calculate_angle(x, y):
    x /= 100  # Convert cm to meters
    y /= 100  # Convert cm to meters
    a = -9.81    # Gravity
    """"
    drag=0.0012

    v = v * (1 - drag * x)
    if v <= 0:
        v = 0.1
    """
    A = (a * x * x) / (2 * v**2)
    B = x
    C = A - (y - sh)  # Adjust for starting height
    
    D = B * B - 4 * A * C
    if D < 0: 
        return 45  # Default angle if no solution
    
    t1 = (-B + math.sqrt(D)) / (2 * A)
    t2 = (-B - math.sqrt(D)) / (2 * A)
    
    theta1 = math.degrees(math.atan(t1))
    theta2 = math.degrees(math.atan(t2))
    
    valid = [t for t in (theta1, theta2) if 0 <= t <= 90]
    if not valid:
        return None

    return sorted(valid)




targets=[[100,0.001,100]]
#targets=[[0,-80,0],[-100,0,0],[0,100,0]]
#targets.reverse()

for i in targets:
    aim(*i)
    print("Aimed at target:", i)
    shoot(6)


wait(2000)


# always include reset for simplicity!!!!
reset()

print("Sequence complete!")
