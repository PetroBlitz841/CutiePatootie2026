from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Axis
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

def button_motor_control(speed=450):
    """Move the right or left motor while the corresponding button is held.

    RIGHT arrow runs the right motor while pressed.
    LEFT arrow runs the left motor while pressed.
    CENTER exits the control loop.
    """
    while True:
        pressed = hub.buttons.pressed()

        if Button.RIGHT in pressed:
            right_motor.run(speed)
        else:
            right_motor.stop()

        if Button.LEFT in pressed:
            left_motor.run(speed)
        else:
            left_motor.stop()

        if Button.CENTER in pressed:
            right_motor.stop()
            left_motor.stop()
            break

        wait(10)

def button_drive_control(speed=600):
    """Drive the robot while the corresponding button is held.

    RIGHT arrow drives forward while pressed.
    LEFT arrow drives backward while pressed.
    CENTER exits the control loop.
    """
    cutie.drive(speed, 0)
    while True:
        pressed = hub.buttons.pressed()

        if Button.CENTER in pressed:
            cutie.stop()
            break

        wait(10)

menu = ['C', 'D']
selected = hub_menu(*menu)  # pylint: disable=assignment-from-no-return

hub.speaker.beep(659, 0.5)   # E5
if selected == "C":
    button_motor_control()
elif selected == "D":
    button_drive_control()