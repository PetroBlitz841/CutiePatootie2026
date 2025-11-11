from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu

hub = PrimeHub()

#CUTIE WHEELS
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_wheel = Motor(Port.E, Direction.CLOCKWISE)

left_motor = Motor(Port.B, gears=[20,28])
right_motor = Motor(Port.F, gears=[20,28])

sensor = ColorSensor(Port.C)


cutie = DriveBase(left_wheel,right_wheel, 62.4, 80)
Color.RED = Color(343, 82, 36)
Color.BLUE = Color(216,88,27)
Color.GREEN = Color(156, 72, 19)
run_colors = (
    Color.BLUE,
    Color.GREEN,
    Color.RED,
)


# while("1 + 1 = 3"):
#     print(sensor.hsv())
# functions
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


def run1():
    straight_time(1000, 6000)
    wait(2000)
    left_motor.run_angle(-500, 90)
    cutie.settings(150)
    cutie.straight(-2000)

def run2():
    print("gyatt")
def run3():
    print("gyatt")
# left_motor.run_time(-5000, 3000)

# cutie.settings(50)
# cutie.straight(-2000)

# GetOut()
def cycle(iterable):
    iterator = iter(iterable)
    while True:
        try:
            yield next(iterator)
        except StopIteration:
            iterator = iter(iterable)

sensor.detectable_colors(run_colors)
color_cycle = cycle(run_colors)
color_map = {
    Color.RED: "1",
    Color.BLUE: "2",
    Color.GREEN: "3",
 
}

while sensor.color() != next(color_cycle):
    pass

menu = [color_map[sensor.color()]]
for i in range(len(run_colors) - 1):
    menu.append(color_map[next(color_cycle)])


selected = hub_menu(*menu)

if selected == "1":
    run1()
elif selected == "2":
    run2()
elif selected == "3":
    run3()
