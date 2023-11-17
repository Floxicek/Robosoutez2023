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
LIFT_ROT = 1050


ev3.screen.set_font(Font(size=15))
ev3.screen.print("Control motor with\n arrow up or down")

lift_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)

# gyro_sensor = GyroSensor(Port.S3)


## Lift motor INIT
ev3.screen.print("Reset belt motor rotation")
lift_motor.run_until_stalled(MAX_SPEED, Stop.BRAKE)
lift_motor.run_until_stalled(-MAX_SPEED, Stop.BRAKE, 96)

wait(300)

lift_motor.reset_angle(0)
# print(lift_motor.control.target_tolerances())

## GSENSOR INIT

# gyro_sensor.reset_angle(0)


# Initialize functions
def elevatorUp():
    lift_motor.run_target(MAX_SPEED, LIFT_ROT)


def elevatorDown():
    lift_motor.run_target(MAX_SPEED, 0)
    ev3.screen.print("Lift motor down")
    ev3.screen.print("{angle} / 0".format(angle=lift_motor.angle()))


def dropCube():
    lift_motor.run_until_stalled(MAX_SPEED, Stop.BRAKE)


while True:
    up_pressed = Button.UP in ev3.buttons.pressed()
    down_pressed = Button.DOWN in ev3.buttons.pressed()
    drop_cube_pressed = Button.CENTER in ev3.buttons.pressed()

    right_pressed = Button.RIGHT in ev3.buttons.pressed()
    left_pressed = Button.LEFT in ev3.buttons.pressed()

    if up_pressed:
        elevatorUp()
    elif down_pressed:
        elevatorDown()
    elif drop_cube_pressed:
        dropCube()
    # ev3.screen.print(lift_motor.angle())
    # ev3.screen.print(gyro_sensor.angle())
