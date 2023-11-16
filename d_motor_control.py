#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Button
from pybricks.media.ev3dev import Font

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# ev3.speaker.beep()

ev3.screen.set_font(Font(size=18))

ev3.screen.print("Control motor with")
ev3.screen.print("arrow up or down")

belt_motor = Motor(Port.D)



# Initialize functions
def elevatorUp():
    belt_motor.run(-1000)


def elevatorDown():
    belt_motor.run(1000)


while True:
    up_pressed = Button.UP in ev3.buttons.pressed()
    down_pressed = Button.DOWN in ev3.buttons.pressed()
    if up_pressed:
        elevatorUp()
    elif down_pressed:
        elevatorDown()
    else:
        belt_motor.stop()


# Go forward and backwards for one meter.
# robot.straight(1000)

# robot.straight(-1000)

# Turn clockwise by 360 degrees and back again.
# robot.turn(360)
# ev3.speaker.beep()

# robot.turn(-360)
# ev3.speaker.beep()


# elevatorUp()
# elevatorDown()


# ev3.speaker.beep()
