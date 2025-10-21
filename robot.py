'''
CUTIE PATOOTIE 2026
'''
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

#CUTIE WHEELS
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_wheel = Motor(Port.E, Direction.CLOCKWISE)

left_motor = Motor(Port.B, gears=[20,28])
right_motor = Motor(Port.F, gears=[20,28])

sensor = ColorSensor(Port.D)


cutie = DriveBase(left_wheel,right_wheel, 62.4, 80)
Color.MAGENTA = Color(340, 80, 56)
sensor.detectable_colors([Color.MAGENTA])

#functions
# def GetOut():
#     cutie.drive(-50, 0)

#     while sensor.color() != Color.MAGENTA:
#         print(sensor.color())
#         print(sensor.hsv())
#         pass

#     cutie.stop()

# right_motor.run_time(5000, 5000, wait=False)

def straight_time(speed, time):
    timer = StopWatch()
    last = cutie.settings()[0]
    cutie.settings(speed)

    while timer.time() < time:
        if speed > 0:
            cutie.straight(1000, wait=False)
        else:
            cutie.straight(-1000, wait=False)
    cutie.stop()

    cutie.settings(last)

straight_time(1000, 6000)
wait(2000)
left_motor.run_angle(-500, 90)
cutie.settings(150)
cutie.straight(-2000)
# left_motor.run_time(-5000, 3000)

# cutie.settings(50)
# cutie.straight(-2000)

# GetOut()
