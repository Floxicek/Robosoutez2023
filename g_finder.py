#!/usr/bin/env pybricks-micropython


from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.media.ev3dev import Font
from pybricks.tools import wait


# Initialize the EV3 Brick.
ev3 = EV3Brick()
# ev3.speaker.beep()



ev3.screen.set_font(Font(size=18))


while True:
    ev3.screen.print(gyro_sensor.angle())
