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
CA = Motor ('C')
def PID(distance, speed, targ = 0, kp = 0.8, ki = 0.005, kd = 0.2):
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
    if angle >= 0:
        while hub.motion_sensor.get_yaw_angle() <= angle+3:
            
            LM.start_at_power(-1 * speed)
            RM.start_at_power(-1 * speed)
    else:
        while hub.motion_sensor.get_yaw_angle() >= angle-3:
            RM.start_at_power(speed)
            LM.start_at_power(speed)
    LM.stop()
    RM.stop()
def LowerArm(speed):
    CA.set_degrees_counted(0)
    CA.run_for_degrees(-370, speed)


def Trip7():
    PID(360, 50)
    turn(45, 50)
    PID(1.8 * 360, 50)
    turn(-45, 50)
    PID(0.75 * 360, 100)
    sleep(0.75)
    PID(0.4 * 360, -50)
    turn(45, 50)
    PID(0.82 * 360, 50)
    turn(30, 50)
    PID(2 * 360, -100)
    PID(0.5 * 360, 100)
    PID(360, -100)
    PID(180, 100)
    PID(360, -100)
    PID(180, 100)
    # turn(45, 50)
    # PID(90, 50)
    # turn(-135, 50)
    # PID(180, 50)
    # sleep(0.1)
    # PID(180, -50)
    hub.status_light.on('azure')


def returnBack():
    turn(-45, 50)
    PID(3.5 * 360, -50)

Trip7()