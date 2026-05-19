from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Axis, Icon
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

cutie.use_gyro(True)

CUSTOM_BLUE = Color(216, 88, 27)

color_list = [CUSTOM_BLUE]
sensor2.detectable_colors(color_list)
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
    till_blue(speed, turn_rate)

    cutie.drive(speed, turn_rate)
    wait_for_stable_roll()
    cutie.stop()


def till_blue(speed, turn_rate):
    """Drive until the blue block on the ramp is detected on the color sensor.
    
    Args:
        speed (int): Speed at which to drive.
        turn_rate (float): Turn rate while driving.
    """
    cutie.drive(speed=speed, turn_rate=turn_rate)  # start driving

    while sensor2.color() != CUSTOM_BLUE:
        wait(10)

    cutie.stop()  # Stop


def button_motor_control(speed=400):
    """Move the right or left motor while the corresponding button is held.

    RIGHT arrow runs the right motor while pressed.
    LEFT arrow runs the left motor while pressed.
    CENTER exits the control loop.
    """
    left_motor.run_time(-speed, 5000, wait=False)  # Run for 2 seconds
    right_motor.run_time(speed, 5000, wait=False)  # Run for 2 seconds
        
    while True:
        pressed = hub.buttons.pressed()
        
        if Button.RIGHT in pressed:
            left_motor.stop()
            right_motor.stop()
            left_motor.run_time(speed, 5000, wait=False)  # Run for 2 seconds
            right_motor.run_time(-speed, 5000)  # Run for 2 seconds

        wait(10)

def button_drive_control(speed=300):
    """Drive the robot while the corresponding button is held.

    RIGHT arrow drives forward while pressed.
    LEFT arrow drives backward while pressed.
    CENTER exits the control loop.
    """
    cutie.settings(600)
    going_down(-100, 0)

menu = ['C', 'D']
selected = hub_menu(*menu)  # pylint: disable=assignment-from-no-return

hub.display.icon(Icon.HAPPY)
hub.speaker.beep(659, 0.5)   # E5
if selected == "C":
    button_motor_control()
elif selected == "D":
    button_drive_control()