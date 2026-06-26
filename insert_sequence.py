import keyboard
import time

current_number = 0
first_press = True


def on_insert(event):
    global current_number, first_press

    if first_press:
        keyboard.write(str(current_number))
        first_press = False
    else:
        keyboard.press_and_release('ctrl+a')
        time.sleep(0.01)
        keyboard.press_and_release('backspace')
        current_number += 1
        keyboard.write(str(current_number))


if __name__ == '__main__':
    value = input('Enter starting number: ').strip()
    if not value:
        print('No number provided. Exiting.')
        raise SystemExit(0)

    current_number = int(value)

    print('Press Insert to type the next number. Press Esc to exit.')
    keyboard.on_press_key('insert', on_insert)
    keyboard.wait('esc')
    print('Stopped.')
