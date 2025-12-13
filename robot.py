from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu

hub = PrimeHub()

# CUTIE WHEELS
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_wheel = Motor(Port.E, Direction.CLOCKWISE)

left_motor = Motor(Port.B, gears=[20, 28])
right_motor = Motor(Port.F, gears=[20, 28])

sensor = ColorSensor(Port.C)
sensor2 = ColorSensor(Port.D)

# while sensor2.reflection() > 20:
#         print(sensor2.reflection())


cutie = DriveBase(left_wheel, right_wheel, 62.4, 80)
color_list = [
    Color(343, 82, 36),  # red
    Color(216, 88, 27),  # blue
    Color(156, 72, 19),  # green
    Color(51, 75, 71),  # yellowSet-ExecutionPolicy Unrestricted -ForceSet-ExecutionPolicy Unrestricted -Force
]
run_colors = (Color.BLUE, Color.GREEN, Color.RED)
sensor2.detectable_colors(color_list)

# while("1 + 1 = 3"):
#     print(sensor2.hsv())
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


def till_black(speed, turn_rate):
    cutie.drive(speed, turn_rate)

    while sensor2.reflection() > 7:
        print(sensor2.reflection())
        pass

    cutie.stop()


# def abs_turn(target):
#     distance = target - cutie.angle()

#     cutie.turn(turnDeg)

# def turn_to(angle):
#     print(hub.imu.heading())
#     start_angle = (hub.imu.heading() + 360) % 360  # 208
#     print(start_angle)
#     deg_to_turn = (angle - start_angle) % 360  # 242
#     print(deg_to_turn)

#     if deg_to_turn >= 180:
#         chassis.turn(deg_to_turn - 360)
#     else:
#         chassis.turn(deg_to_turn)

# def turn_to_angle(angle, speed=200, max_time=3):
#     """Turns to a specified absolute gyro angle"""

#     timer = StopWatch()
#     timer.reset()

#     distance = angle - hub.imu.heading()
#     robot_acceleration = wheels.settings()[3]
#     wheels.settings(turn_rate=speed)
#     wheels.turn(distance)

#     # while (timer.time()) < (max_time * 1000) and angle - hub.imu.heading() > 1:
#     #     ...
#     wheels.settings(turn_rate=robot_acceleration)


def is_color_in_range(measured_color: Color, comparison_color: Color, range: int):
    h = comparison_color.h - range <= measured_color.h <= comparison_color.h + range
    s = comparison_color.s - range <= measured_color.s <= comparison_color.s + range
    v = comparison_color.v - range <= measured_color.v <= comparison_color.v + range

    return h & s & v


def GetOut():
    cutie.drive(-50, 0)

    # while not is_color_in_range(sensor2.color, yellow, 10):
    while sensor2.color != Color(52, 75, 77):
        # print(sensor2.hsv())
        print(sensor2.hsv())
        pass

    left_wheel.hold()
    right_motor.hold()


# GetOut()
# till_black(100, 0)
def run1():
    cutie.settings(700)
    cutie.straight(1300)
    straight_time(300, 4000)
    wait(2000)
    cutie.settings(150)
    cutie.straight(-500)
    cutie.turn(90)
    cutie.straight(-500)


def run2():
    cutie.use_gyro(True)
    cutie.settings(80)
    cutie.straight(700)
    right_motor.run_time(-270, 1000)
    cutie.straight(-600)



def run3():
    cutie.straight(700)
    till_black(100, 0)
    cutie.turn(57)
    cutie.straight(300)
    right_motor.run_time(-5000, 1000)
    cutie.straight(-300)
    cutie.turn(33)
    cutie.straight(200)


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
