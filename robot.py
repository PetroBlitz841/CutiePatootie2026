from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop
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


def wait_for_right_arrow():
    """Wait until the right arrow button is pressed on the hub."""
    while Button.RIGHT not in hub.buttons.pressed():
        wait(100)


def straight_time(speed, time):
    """Move straight for a specified duration.
    
    Args:
        speed (int): Speed at which to move (positive or negative).
        time (int): Duration to move in milliseconds.
    """
    cutie.drive(speed, 0)
    wait(time)
    cutie.stop()


def turn_time(turn_rate, time):
    """Turns the robot in place for a specified duration.

    Args:
        turn_rate (int): Turn rate to apply. Positive values turn right, negative values turn left.
        time (int): Duration of the turn in milliseconds.
    """
    cutie.drive(0, turn_rate)
    wait(time)
    cutie.stop()


def curve_time(time, turn_rate):
    """Drive in a curve for a specified duration.
    
    Args:
        time (int): Duration to drive in milliseconds.
        angle (float): Turn rate for the curve.
    """
    speed = cutie.settings()[0]
    cutie.drive(speed, turn_rate)
    wait(time)
    cutie.stop()


def till_black(speed, turn_rate):
    """Drive until a black line is detected on the color sensor.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    cutie.drive(speed=speed, turn_rate=turn_rate)  # start driving

    while sensor2.reflection() > 7:  # while refelction over 7, continue driving
        wait(10)

    cutie.stop()  # Stop


def till_yellow(speed, turn_rate):
    """Drive until the yellow block on the ramp is detected on the color sensor.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    cutie.drive(speed=speed, turn_rate=turn_rate)  # start driving

    while sensor2.color() != CUSTOM_YELLOW:
        wait(10)

    cutie.stop()  # Stop

def wait_for_stable_roll(window_size=10, poll_ms=10, tolerance=1):
    """Poll the robot till continuously and keep the last
    `window_size` readings in a FIFO list. When the average of the
    window is within the tolerance, return the averaged value.

    Args:
        window_size (int): number of readings to keep (default 10).
        poll_ms (int): milliseconds to wait between polls (default 10).
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
                return avg

        wait(poll_ms)


def going_down(speed, turn_rate):
    """Drive to the yellow line and wait for the robot to stabilize.
    
    Drives until the yellow line is detected, then continues driving while
    waiting for the roll (tilt) to stabilize before continuing.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    till_yellow(speed, turn_rate)

    cutie.drive(speed, turn_rate)
    wait_for_stable_roll()
    cutie.stop()


def gyro_turn(
    target,
    max_rate=150,
    kp=2.1,
    kd=0.6,
    ke=16,
    angle_tol=0.3,
    speed_tol=30,
    max_time=2670,
):
    """Turn the robot to a target heading using gyro-based PD control.
    
    Uses the IMU gyroscope and PD (proportional-derivative) control with a static
    bias to overcome motor friction and achieve precise heading control.
    
    Args:
        target (float): Target heading in degrees.
        max_rate (int): Maximum turn rate in degrees/second (default 150).
        kp (float): Proportional gain constant.
        kd (float): Derivative gain constant.
        ke (int): Static bias constant to overcome motor friction.
        angle_tol (float): Angle tolerance threshold in degrees (default 0.3).
        speed_tol (int): Turn rate tolerance threshold (default 30).
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
        turn_rate += ke if turn_rate > 0 else -ke

        # Clamp turn rate
        turn_rate = max(-max_rate, min(max_rate, turn_rate))
        cutie.drive(0, turn_rate)

        # Exit condition: close enough and slow enough
        if abs(error) < angle_tol and abs(turn_rate) < speed_tol:
            break

        last_error = error
        wait(10)  # smaller wait for faster updates
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


# cheks for the turn of the robot PD vs P
# timer = StopWatch()
# timer.reset()
# gyro_turn(90)
# print(hub.imu.heading())
# print(timer.time())
# timer.reset()
# gyro_abs(180)
# print(hub.imu.heading())
# print(timer.time())

# left_motor.run_time(2000, 2000)
def run1():
    """Execute the first robot run sequence.
    """
    # MERKAVA!!!!!
    # cutie.settings(straight_speed = 1000) #set speed to 1000
    # cutie.straight(distance=1300, then=Stop.NONE) #Go straight
    # straight_time(speed = 1000, time = 3000) #straight time

    # cutie.settings(150, turn_rate=40)  # apply settings
    # cutie.use_gyro(True)

    # GOING DOWN
    # cutie.settings(600)
    # cutie.straight(-100) # speedy straight before controlled descent
    # going_down(-100, 0)
    # gyro_turn(0)
    # cutie.settings(200)
    # cutie.straight(-50)
    # cutie.settings(300)
    # cutie.curve(-450, -30)
    # gyro_turn(0)
    # cutie.settings(400)
    # cutie.straight(-530)
    # cutie.settings(turn_acceleration=200)
    # cutie.use_gyro(False)
    # cutie.settings(200, turn_rate=400)
    cutie.turn(90)
    left_motor.run_time(speed=-300, time=3000, wait=False)
    straight_time(-250, 1600)
    right_motor.run_time(speed=-300, time=3000, wait=False)
    hub.imu.reset_heading(90)

    cutie.settings(turn_rate=70, straight_speed=80)
    cutie.straight(20)
    gyro_turn(0)
    cutie.use_gyro(True)
    till_black(100, 0)
    cutie.straight(-10)
    right_motor.run_time(300, 2000)
    right_motor.run_time(-300, 2000)
    left_motor.run_time(2000, 2000)

    cutie.settings(turn_rate=70, straight_speed=150)
    gyro_turn(0)
    cutie.straight(-170)
    right_motor.run_time(1000, 2000, wait=False)
    left_motor.run_time(1000, 5000, wait=False)
    wait(2000)
    cutie.straight(120)
    gyro_abs(0, 250, ke=25)
    cutie.straight(-120)
    cutie.straight(80)
    cutie.turn(-30)
    cutie.straight(50)
    cutie.straight(-110)
    cutie.settings(straight_speed=400, turn_rate=200)
    turn_to(-80)
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
    cutie.straight(-450, then=Stop.NONE)  # go back
    cutie.use_gyro(True)
    cutie.curve(500, -25, then=Stop.NONE)
    cutie.straight(450)
    gyro_turn(0)
    cutie.straight(60)
    till_black(100, 0)  # go to black line
    cutie.settings(straight_speed=100, turn_rate=80)
    cutie.straight(45)
    gyro_abs(-90, ke=25) #turn to mission
    till_black(-90, 0) #go to misiion using black line
    cutie.straight(-40)
    right_motor.run_time(-1000, 2800, wait=False) #turn gear (lift up items)
    turn_time(-30, 2800) #turn to gear while turning
    cutie.use_gyro(True)
    cutie.straight(35)
    gyro_abs(-90, ke=25) #fix up
    cutie.settings(200)
    till_black(100, 0)
    cutie.straight(100)
    gyro_turn(0)
    till_black(-100, 0)
    cutie.straight(195)
    cutie.straight(-100)
    cutie.settings(300)
    cutie.turn(45)
    cutie.straight(50, then=Stop.NONE)
    cutie.curve(60, -45, then=Stop.NONE)
    cutie.straight(470)
    right_motor.run_time(-200, 2000, wait=False)
    gyro_abs(45)
    cutie.straight(-200)
    right_motor.run_time(200, 3000, wait=False)
    left_motor.run_time(1500, 3000)
    cutie.straight(200)
    cutie.straight(-50)
    left_motor.run_time(-500, 2000)
    cutie.straight(1000)


def run3():
    """Execute the third robot run sequence.
    """
    cutie.use_gyro(True)
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
    left_motor.run_time(speed=90, time=1500)  # forum, mechanical stop
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

    hub.speaker.beep(659, 10000) 
    cutie.drive(0, 360)
    while True:
        wait(100)


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
run_map = {str(i+1): func for i, func in enumerate([run1, run2, run3, run4])}
color_map = {color: str(i + 1) for i, color in enumerate(run_colors)}

color_cycle = cycle(run_colors)

while sensor.color() != next(color_cycle):
    pass

menu = [color_map[current_color]] + [color_map[next(color_cycle)] for _ in range(len(run_colors) - 1)]
selected = hub_menu(*menu)  # pylint: disable=assignment-from-no-return

hub.speaker.beep(659, 0.5)   # E5
hub.imu.reset_heading(0)

run_map[selected]()
