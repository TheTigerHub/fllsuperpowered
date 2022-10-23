from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from spike.operator import equal_to
from time import sleep
from math import *

hub = PrimeHub()

# Sensors
colorF = ColorSensor('F')
# Motors
motors = MotorPair('E', 'A')
LM = Motor('E')
RM = Motor('A')
DA = Motor('D')
CA = Motor('C')
# Functions
def PID(distance, speed, targ = 0, kp = 0.5, ki = 0.01, kd = 0.3):
    hub.motion_sensor.reset_yaw_angle()
    LM.set_degrees_counted(0)
    RM.set_degrees_counted(0)

    curErr = 0
    prevErr = 0
    accErr = 0
    DerErr = 0
    P = 0
    I = 0
    D = 0
    PID = 0
    if speed > 0:
        while LM.get_degrees_counted() >= -1 * distance:
            speed = speed
            curErr = targ - hub.motion_sensor.get_yaw_angle()
            accErr += curErr
            DerErr = curErr - prevErr
            prevErr = curErr
            P = kp * curErr
            I = ki * accErr
            D = kd * DerErr
            PID = P + I + D
            motors.start_tank_at_power(floor(speed + PID), floor(speed - PID))
    else:
        while LM.get_degrees_counted() <= distance:
            speed = speed
            curErr = targ - hub.motion_sensor.get_yaw_angle()
            accErr += curErr
            DerErr = curErr - prevErr
            prevErr = curErr
            P = kp * curErr
            I = ki * accErr
            D = kd * DerErr
            PID = P + I + D
            motors.start_tank_at_power(floor(speed + PID), floor(speed - PID))
    motors.stop()

def turn(angle, speed):
    hub.motion_sensor.reset_yaw_angle()
    if angle > 0:
        while hub.motion_sensor.get_yaw_angle() <= angle-3:
            LM.start_at_power(-1 * speed)
            RM.start_at_power(-1 * speed)
    else:
        while hub.motion_sensor.get_yaw_angle() >= angle+3:
            LM.start_at_power(speed)
            RM.start_at_power(speed)
    LM.stop()
    RM.stop()
def Main2():
    PID(3.8*360, 50)
    turn(15, 50)
    DA.run_for_degrees(95, 50)
    sleep(1)
    DA.run_for_degrees(-95, 50)
    turn(-15, 50)
    PID(160, -50)
    turn(90, 50)
    PID(3.3*360, 50)
    turn(-90, 50)
    PID(135, -50)
    turn(-15, 50)

Main2()
# turn(-15, 50)