from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, hub_menu, wait

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
def gyro_abs(target, speed):
    while not (target > hub.imu.heading() - 0.01) or not (target < hub.imu.heading() + 0.01): # the stoping range
        direction = (target - (hub.imu.heading() % 360)) % 360 # calculating the direction of the turn
        if direction > 180:
            left_wheel.dc(-speed)
            right_wheel.dc(speed)
        else:
            left_wheel.dc(speed)
            right_wheel.dc(-speed)
    cutie.stop()

color_list = [
    Color(343, 82, 36),  # red
    Color(216, 88, 27),  # blue
    Color(156, 72, 19),  # green
    Color(51, 75, 71),  # yellow
]
run_colors = (Color.BLUE, Color.GREEN, Color.RED)
sensor2.detectable_colors(color_list)

cutie.use_gyro(True)
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
        # print(sensor2.reflection())
        pass

    cutie.stop()

color_list2 = [
    Color(156, 72, 19),  # red
]
sensor2.detectable_colors(color_list2)
def till_red(speed, turn_rate):
    cutie.drive(speed, turn_rate)

    while hub.imu.tilt()[1] < -4 or hub.imu.tilt()[1] > 4:
        print(hub.imu.tilt())
        pass

    cutie.stop()
# till_red(-50, 0)


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


def is_color_in_range(measured_color: Color, comparison_color: Color, range: int): # pylint: disable=redefined-builtin
    h = comparison_color.h - range <= measured_color.h <= comparison_color.h + range
    s = comparison_color.s - range <= measured_color.s <= comparison_color.s + range
    v = comparison_color.v - range <= measured_color.v <= comparison_color.v + range

    return h & s & v

def turn_to(angle, then=Stop.HOLD):
    print(hub.imu.heading())
    start_angle = (hub.imu.heading() + 360) % 360  # 208
    print(start_angle)
    deg_to_turn = (angle - start_angle) % 360  # 242
    print(deg_to_turn)
    if then == Stop.NONE:
        if deg_to_turn >= 180:
            cutie.turn(deg_to_turn - 360)
        else:
            cutie.turn(deg_to_turn)
        return
    if deg_to_turn >= 180:
        cutie.turn(deg_to_turn - 360)
    else:
        cutie.turn(deg_to_turn)
# cutie.settings(turn_rate=40)
# cutie.turn(360)
# # turn_to(90)
# # hub.imu.reset_heading(0)
# # wait(50)
# gyro_abs(57, 40)
# gyro_abs(0, 40)

def GetOut():
    cutie.drive(-50, 0)

    # while not is_color_in_range(sensor2.color, yellow, 10):
    while sensor2.color() != Color(52, 75, 77):
        # print(sensor2.hsv())
        print(sensor2.hsv())

    left_wheel.hold()
    right_motor.hold()

# left_motor.run_time(1000, 5000, wait = False)
# cutie.straight(50)
# left_motor.run_time(100, 5000, wait=False)
# cutie.turn(120)
# cutie.curve(radius=600, angle=-30)
# cutie.straight(150)
# # right_motor.run_time(300, 3000)
# left_motor.run_time(-5000, 5000)
# left_motor.run_time(5000, 5000)





# cutie.straight(-3000)
# wait(5000)
# cutie.straight(200)
# cutie.straight(-2000)
# # GetOut()
# till_black(100, 0)
# right_motor.run_time(300, 5000)
# left_motor.run_time(-300, 5000)
    # right_motor.run_time(300, 5000)
# cutie.curve(-6000, -60)
def run1():
    cutie.settings(1000)
    cutie.straight(1300)
    straight_time(600, 4000)
    wait(100)
    cutie.settings(150, turn_rate= 40)
    cutie.use_gyro(True)

    # GOING DOWN
    cutie.settings(80)
    cutie.straight(-400, then=Stop.NONE)
    cutie.settings(400)
    cutie.straight(-100, then=Stop.NONE)
    cutie.curve(-500, -30)
    gyro_abs(0, 30)
    cutie.straight(-480)
    right_motor.run_time(300, 5000, wait=False)
    cutie.use_gyro(False)
    cutie.settings(turn_rate=80)
    turn_to(90)
    straight_time(-200, 2000)
    cutie.use_gyro(True)
    cutie.settings(turn_rate=70, straight_speed= 80)
    right_motor.run_angle(-160, 150, wait=False)
    cutie.straight(25)
    cutie.turn(-90)
    till_black(65, 0)
    cutie.straight(10)
    left_motor.run_time(-2000, 3000, wait=False)
    right_motor.run_angle(-160, 400)
    right_motor.run_angle(160, 220)
    cutie.settings(turn_rate=70, straight_speed= 80)
    cutie.straight(-170)
    right_motor.run_angle(-800, 500)
    cutie.straight(80)
    cutie.straight(-80)
    cutie.straight(160)
    cutie.settings(straight_speed= 400, turn_rate=200)
    cutie.turn(-70)
    cutie.settings(300)
    cutie.straight(-200, then=Stop.NONE)
    cutie.curve(-1000, -60)
    # yiftach was here, dont tell anyone


def run2():
    cutie.use_gyro(True)
    cutie.settings(80)
    for _ in range(4):
        right_motor.run_time(-100,500)
        right_motor.run_time(100,500)





def run3():
    cutie.settings(straight_speed=1000)
    cutie.straight(700)
    till_black(100, 0)
    gyro_abs(48, 30)
    cutie.settings(straight_speed=200)
    cutie.straight(300)
    cutie.straight(-20)
    right_motor.run_time(-5000, 1000) # statue
    left_motor.run_time(90, 1500) # forum, mechanical stop
    cutie.straight(-300) # gets out

    # victory_dance()

def victory_dance():
    # TODO: add rythem and music
    # go to another place to not destroy back lines (turn than drive)
    cutie.drive(0, 360)
    while True:
        wait(100)

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


selected = hub_menu(*menu) # pylint: disable=assignment-from-no-return

if selected == "1":
    hub.imu.reset_heading(0)
    run1()
elif selected == "2":
    hub.imu.reset_heading(0)
    run2()
elif selected == "3":
    hub.imu.reset_heading(0)
    run3()
