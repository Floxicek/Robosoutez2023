#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.media.ev3dev import Font
from pybricks.tools import wait


# Initialize the EV3 Brick.
ev3 = EV3Brick()
ev3.speaker.beep()


MAX_SPEED = 1500
LIFT_ROT = 900


ev3.screen.set_font(Font(size=15))
ev3.screen.print("Control motor with\n arrow up or down")

lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
# gyro_sensor = GyroSensor(Port.S1)


## Lift motor INIT
ev3.screen.print("Reset belt motor rotation")
lift_motor.run_until_stalled(MAX_SPEED, Stop.BRAKE)
lift_motor.run_until_stalled(-MAX_SPEED, Stop.BRAKE)

wait(300)

lift_motor.reset_angle(0)
# print(lift_motor.control.target_tolerances())


## GSENSOR INIT
# gyro_sensor.reset_angle(0)


# Initialize functions
def elevatorUp():
    lift_motor.run_target(MAX_SPEED, LIFT_ROT)


def elevatorDown():
    # lift_motor.run_target(MAX_SPEED, 0)
    lift_motor.run_until_stalled(-MAX_SPEED, Stop.BRAKE)
    ev3.screen.print("Lift motor down")
    ev3.screen.print(lift_motor.angle() + " / 0")


def dropCube():
    lift_motor.run_until_stalled(MAX_SPEED, Stop.BRAKE)


while True:
    up_pressed = Button.UP in ev3.buttons.pressed()

    down_pressed = Button.DOWN in ev3.buttons.pressed()
    drop_cube_pressed = Button.CENTER in ev3.buttons.pressed()

    if up_pressed:
        elevatorUp()
    elif down_pressed:
        elevatorDown()
    elif drop_cube_pressed:
        dropCube()

    # ev3.screen.print(lift_motor.angle())
#   ev3.screen.print(gyro_sensor.angle())
