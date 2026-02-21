from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, hub_menu, wait

hub = PrimeHub()

# CUTIE WHEELS
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # purple
right_wheel = Motor(Port.E, Direction.CLOCKWISE)  # red

left_motor = Motor(Port.F, gears=[20, 28])  # yellow
right_motor = Motor(Port.D, gears=[20, 28])  # blue

sensor = ColorSensor(Port.C)  # green
sensor2 = ColorSensor(Port.B)  # cyan


cutie = DriveBase(left_wheel, right_wheel, wheel_diameter=62.4, axle_track=80)

CUSTOM_RED = Color(343, 82, 36)
CUSTOM_BLUE = Color(216, 88, 27)
CUSTOM_GREEN = Color(156, 72, 19)
CUSTOM_YELLOW = Color(51, 75, 71)

color_list = [CUSTOM_RED, CUSTOM_YELLOW, CUSTOM_BLUE, CUSTOM_GREEN]

run_colors = (Color.RED, Color.YELLOW, Color.BLUE, Color.GREEN)
sensor2.detectable_colors(color_list)

cutie.use_gyro(True)


def straight_time(speed, time):
    """Move straight for a specified duration.
    
    Args:
        speed (int): Speed at which to move (positive or negative).
        time (int): Duration to move in milliseconds.
    """
    timer = StopWatch()  # Start stopwatch
    last = cutie.settings()[0]  # saves speed
    cutie.settings(speed)  # define speed

    while timer.time() < time:  # while timer not reached, go straight
        if speed > 0:
            cutie.straight(1000, wait=False)
        else:
            cutie.straight(-1000, wait=False)
    cutie.stop()

    cutie.settings(last)  # return speed


def curve_time(time, angle):
    """Drive in a curve for a specified duration.
    
    Args:
        time (int): Duration to drive in milliseconds.
        angle (float): Turn rate for the curve.
    """
    timer = StopWatch()  # Start stopwatch

    while timer.time() < time:  # while time not reached, go curve
        cutie.drive(speed=cutie.settings()[0], turn_rate=angle)
    cutie.stop()  # cutie stops


def till_black(speed, turn_rate):
    """Drive until a black line is detected on the color sensor.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    cutie.drive(speed=speed, turn_rate=turn_rate)  # start driving

    while sensor2.reflection() > 7:  # while refelction over 7, continue driving
        pass

    cutie.stop()  # Stop


def wait_for_stable_roll(window_size=10, poll_ms=10, tolerance=1):
    """Poll `hub.imu.tilt()[1]` continuously and keep the last
    `window_size` readings in a FIFO list. When the average of the
    window is within [-tol, tol], stop the robot (`cutie.stop()`) and
    return the averaged value.

    Args:
        window_size (int): number of readings to keep (default 10).
        poll_ms (int): milliseconds to wait between polls (default 30).
        tol (float): tolerance threshold for average (default 1).
    Returns:
        float: the average of the last `window_size` readings when stopped.
    """
    readings = []
    while True:
        r = hub.imu.tilt()[1]
        readings.append(r)
        if len(readings) > window_size:
            readings.pop(0)

        if len(readings) == window_size:
            avg = sum(readings) / float(window_size)
            # print("tilt_avg:", avg)
            if -tolerance <= avg <= tolerance:
                cutie.stop()
                return avg

        wait(poll_ms)


def till_yellow(speed, turn_rate):
    """Drive until the yellow block on the ramp is detected on the color sensor.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    cutie.drive(speed=speed, turn_rate=turn_rate)  # start driving

    while sensor2.color() != CUSTOM_YELLOW:
        pass

    cutie.stop()  # Stop


def going_down(speed, turn_rate):
    """Drive to the yellow line and wait for the robot to stabilize.
    
    Drives until the yellow line is detected, then continues driving while
    waiting for the roll (tilt) to stabilize before continuing.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    till_yellow(speed, turn_rate)
    print("yellow")
    cutie.drive(speed, turn_rate)
    wait_for_stable_roll()
    print("roll 0")


# old
# def gyro_turn(
#     target,
#     max_rate=1000,
#     kp=3.0,
#     kd=0.8,
#     min_rate=10,
#     angle_tol=0.3,
#     speed_tol=10000,
#     max_time=2670
# ):
#     last_error = 0
#     dt = 0.01
#     timer = StopWatch()
#     timer.reset()

#     while timer.time() < max_time:
#         current = hub.imu.heading()

#         # Shortest angle wraparound
#         error = ((target - current + 180) % 360) - 180
#         d_error = (error - last_error) / dt

#         # Dynamic max turn rate based on remaining error
#         dynamic_max = max(min_rate, min(max_rate, abs(error) * 3))

#         # PD control
#         turn_rate = kp * error + kd * d_error

#         # Clamp turn rate
#         turn_rate = max(-dynamic_max, min(dynamic_max, turn_rate))

#         cutie.drive(0, turn_rate)

#         # Exit condition
#         if abs(error) < angle_tol and abs(turn_rate) < speed_tol:
#             break

#         last_error = error
#         wait(10)

#     cutie.stop()
#     wait(200)


def gyro_turn(
    target,
    max_rate=1000,
    kp=5.0,
    kd=0.6,
    ke=20,
    angle_tol=0.3,
    speed_tol=100,
    max_time=2670,
):
    """Turn the robot to a target heading using gyro-based PD control.
    
    Uses the IMU gyroscope and PD (proportional-derivative) control with a static
    bias to overcome motor friction and achieve precise heading control.
    
    Args:
        target (float): Target heading in degrees.
        max_rate (int): Maximum turn rate in degrees/second (default 1000).
        kp (float): Proportional gain constant (default 5.0).
        kd (float): Derivative gain constant (default 0.6).
        ke (int): Static bias to overcome motor friction (default 20).
        angle_tol (float): Angle tolerance threshold in degrees (default 0.3).
        speed_tol (int): Turn rate tolerance threshold (default 100).
        max_time (int): Maximum time to attempt turn in milliseconds (default 2670).
    """

    last_error = 0
    timer = StopWatch()
    timer.reset()

    last_time = timer.time()

    while timer.time() < max_time:
        current = hub.imu.heading()

        # Shortest angle wraparound
        error = ((target - current + 180) % 360) - 180

        # Real dt
        now = timer.time()
        dt = (now - last_time) / 1000.0  # convert ms to seconds
        last_time = now

        if dt == 0:
            dt = 0.001  # avoid division by zero

        # PD control
        d_error = (error - last_error) / dt
        turn_rate = kp * error + kd * d_error

        # Add static bias to overcome motor deadzone
        if turn_rate > 0:
            turn_rate += ke
        elif turn_rate < 0:
            turn_rate -= ke

        # Clamp turn rate
        turn_rate = max(-max_rate, min(max_rate, turn_rate))

        cutie.drive(0, turn_rate)

        # Exit condition: close enough and slow enough
        if abs(error) < angle_tol and abs(turn_rate) < speed_tol:
            break

        last_error = error
        wait(10)  # smaller wait for faster updates
    print(timer.time())
    cutie.stop()
    wait(200)


def turn_to(angle, then=Stop.HOLD):
    """Turn the robot to a specific absolute angle.
    
    Uses PyBricks' built-in turn function to rotate to the target angle.
    
    Args:
        angle (float): Target absolute angle in degrees.
        then (Stop): Stop behavior after turning (default Stop.HOLD).
    """
    start_angle = (hub.imu.heading() + 360) % 360  # cal
    deg_to_turn = (angle - start_angle) % 360  # calculate how much need to turn
    if then == Stop.NONE:
        if deg_to_turn >= 180:
            cutie.turn(angle=deg_to_turn - 360)
        else:
            cutie.turn(angle=deg_to_turn)
        return
    if deg_to_turn >= 180:
        cutie.turn(deg_to_turn - 360)
    else:
        cutie.turn(deg_to_turn)


def gyro_abs(target_angle, kp=1.5, ke=20):
    """Turn to an absolute angle using gyro-based proportional control.
    
    Args:
        target_angle (float): Target absolute angle in degrees.
        kp (float): Proportional gain constant (default 1.5).
        ke (int): Static bias to overcome motor friction (default 20).
    """

    while True:
        error = (
            (target_angle - hub.imu.heading() + 180) % 360
        ) - 180  # calculate error

        turn_rate = error * kp  # calculate turning speed

        if turn_rate > 0:
            turn_rate += ke  # apply ks(kavua stati)
        else:
            turn_rate -= ke  # apply ks(kavua stati)

        if abs(error) < 0.3:  # when reached reasonable error, exit loop
            break

        cutie.drive(0, turn_rate=turn_rate)  # apply speed
        wait(10)
    cutie.stop()  # stop
    wait(200)


# cheks for the turn of the robot PD vs SHITTERY
# timer = StopWatch()
# timer.reset()
# gyro_turn(90)
# print(hub.imu.heading())
# print(timer.time())
# timer.reset()
# gyro_abs(180)
# print(hub.imu.heading())
# print(timer.time())


def run1():
    """Execute the first robot run sequence.
    """
    # MERKAVA!!!!!
    cutie.settings(straight_speed = 1000) #set speed to 1000
    cutie.straight(distance=1300, then=Stop.NONE) #Go straight
    straight_time(speed = 1000, time = 3000) #straight time
    cutie.settings(150, turn_rate=40)  # apply settings
    cutie.use_gyro(True)

    # GOING DOWN
    cutie.settings(200)
    going_down(-100, 0)

    cutie.straight(-50)
    gyro_turn(0)
    cutie.settings(300)
    cutie.curve(-450, -30)
    gyro_turn(0)
    cutie.settings(400)
    cutie.straight(-550)
    cutie.settings(turn_acceleration=200)
    cutie.use_gyro(False)
    cutie.settings(200, turn_rate=400)
    cutie.turn(90)
    right_motor.run_time(speed=300, time=3000, wait=False)
    straight_time(-250, 1600)
    hub.imu.reset_heading(90)

    cutie.settings(turn_rate=70, straight_speed=80)
    cutie.straight(20)
    gyro_turn(0, kp=3)
    cutie.use_gyro(True)
    till_black(100, 0)
    cutie.straight(10)
    left_motor.run_time(-2000, 3500, wait=False)
    right_motor.run_angle(-160, 600)
    right_motor.run_until_stalled(800, duty_limit=30)
    right_motor.run_angle(-160, 120)
    cutie.settings(turn_rate=70, straight_speed=150)
    gyro_abs(0, 250, ke=25)
    cutie.straight(-170)
    right_motor.run_angle(-800, 500)
    cutie.straight(120)
    gyro_abs(0, 250, ke=25)
    cutie.straight(-120)
    cutie.straight(80)
    cutie.turn(-30)
    cutie.straight(50)
    cutie.straight(-110)
    cutie.settings(straight_speed=400, turn_rate=200)
    turn_to(-90)
    cutie.settings(1000, turn_rate=1000)
    cutie.curve(-700, -60, then=Stop.NONE)
    cutie.straight(-600)
    # yiftach was here, dont tell anyone


def run2():
    """Execute the second robot run sequence.
    """
    curve_time(3000, 5)  # go into wall and into boat
    right_motor.run_time(-1000, 1000)  # Drop flag
    cutie.settings(400)
    cutie.straight(-500, then=Stop.NONE)  # go back
    cutie.use_gyro(True)
    cutie.curve(500, -25, then=Stop.NONE)
    cutie.straight(450)
    gyro_abs(0, 250, ke=25)
    cutie.straight(50)
    till_black(100, 0)  # go to black line
    cutie.settings(straight_speed=100, turn_rate=80)
    cutie.straight(50)
    gyro_abs(-90, 250, ke=25)  # turn to mission
    till_black(-100, 0)  # go to misiion using black line
    cutie.straight(-40)
    cutie.turn(-30)  # turn to gear
    right_motor.run_time(-1000, 3000)  # turn gear (lift up items)
    cutie.use_gyro(True)
    gyro_abs(-90, 250, ke=25)  # fix up
    cutie.settings(80)
    cutie.straight(30)
    till_black(100, 0)
    cutie.straight(100)
    gyro_abs(0, 250, ke=20)
    till_black(-100, 0)
    cutie.straight(195)
    cutie.straight(-100)
    cutie.settings(210)
    cutie.turn(45)
    cutie.straight(40, then=Stop.NONE)
    cutie.curve(60, -45, then=Stop.NONE)
    cutie.straight(570)
    right_motor.run_time(-200, 2000, wait=False)
    gyro_abs(45)
    cutie.straight(-200)
    cutie.straight(10)
    right_motor.run_time(200, 3000, wait=False)
    left_motor.run_time(1500, 3000)
    cutie.straight(200)
    cutie.straight(-50)
    left_motor.run_time(-500, 2000)
    cutie.straight(1000)


def run3():
    """Execute the third robot run sequence.
    """
    cutie.settings(straight_speed=1000)  #
    cutie.straight(distance=300, then=Stop.NONE)  # go straight
    cutie.settings(straight_speed=300)
    cutie.straight(440, then=Stop.NONE)
    till_black(speed=100, turn_rate=0)  # until black
    gyro_abs(45, ke=5, kp=2)  # turn to degree 45
    cutie.settings(straight_speed=320, straight_acceleration=750, turn_rate=250)
    cutie.straight(300)  # go into statue
    cutie.straight(-25)
    right_motor.run_time(speed=-5000, time=1000)  # statue
    left_motor.run_time(speed=300, time=1500)  # forum, mechanical stop
    cutie.turn(-25)
    cutie.straight(-300)  # gets out

    # victory_dance()


def run4():
    """Execute the fourth robot run sequence.
    """
    cutie.settings(straight_speed=500, straight_acceleration=350)
    left_motor.run_until_stalled(-1100)
    cutie.straight(650)
    cutie.straight(-250)
    cutie.straight(150)
    left_motor.run_time(200, 5000, wait=False)
    wait(1500)
    cutie.straight(-400, then=Stop.NONE)
    cutie.curve(-200, -60, then=Stop.NONE)
    cutie.straight(-4000)


def victory_dance():
    """Execute a victory dance after finishing all missions.
    """
    # TODO: add rythem and music
    # go to another place to not destroy back lines (turn than drive)
    cutie.drive(0, 360)
    while True:
        wait(100)


# GetOut()
def cycle(iterable):
    """Continuously cycle through an iterable, restarting when exhausted.
    
    Args:
        iterable: An iterable to cycle through.
        
    Yields:
        Elements from the iterable, repeating infinitely.
    """
    iterator = iter(iterable)
    while True:
        try:
            yield next(iterator)
        except StopIteration:
            iterator = iter(iterable)


sensor.detectable_colors(run_colors)
color_cycle = cycle(run_colors)
color_map = {Color.RED: "1", Color.BLUE: "2", Color.GREEN: "3", Color.YELLOW: "4"}

while sensor.color() != next(color_cycle):
    pass

menu = [color_map[sensor.color()]]
for i in range(len(run_colors) - 1):
    menu.append(color_map[next(color_cycle)])


selected = hub_menu(*menu)  # pylint: disable=assignment-from-no-return

if selected == "1":
    hub.imu.reset_heading(0)
    run1()
elif selected == "2":
    hub.imu.reset_heading(0)
    run2()
elif selected == "3":
    hub.imu.reset_heading(0)
    run3()
elif selected == "4":
    hub.imu.reset_heading(0)
    run4()
