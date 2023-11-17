#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick

from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()

watch = StopWatch()


while True:
    print(watch.status())
    watch.pause()
    print(watch.status())
    wait(100)
    watch.resume()
