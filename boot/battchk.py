#!/usr/bin/python3
# coding=utf8
import sys
import signal
sys.path.append('/home/pi/TurboPi/')
sys.path.append('/home/pi/boot/')

import os
import time
import argparse

import RPi.GPIO as GPIO
import HiwonderSDK.Board as Board
import HiwonderSDK.Sonar as Sonar


import warnings
try:
    import buttonman as buttonman
except ImportError:
    buttonman = None
    warnings.warn("buttonman was not imported, so no processes can be registered. This means the process can't be stopped by buttonman.",  # noqa: E501
                  ImportWarning, stacklevel=2)

sys.stdout.reconfigure(encoding="utf-8")


#######

key1_debouncer = None
key2_debouncer = None

environ_silent = os.environ.get('silent', None)
do_beeps = environ_silent is None or environ_silent.lower() == 'false'
do_flash = True

BUZZER_PIN = 31  # board pin numbering
KEY1_PIN = 33
KEY2_PIN = 16
KDN = GPIO.LOW
KUP = GPIO.HIGH

BAD_CELL_VOLTAGE = 3.1

n = 10
__stop = False
button_listen = False
button_states = [KUP, KUP]

s = Sonar.Sonar()

rgb = {
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'lime': (80, 222, 0),
    'yellow': (200, 200, 0),
    'orange': (255, 50, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

spin_period = 0.100


def waitif(t):
    n = t / spin_period
    for _ in range(int(n)):
        if __stop:
            return True
        time.sleep(spin_period)
    time.sleep(n % spin_period)
    if __stop:
        return True


def buzzer(value):
    if do_beeps:
        GPIO.output(BUZZER_PIN, int(bool(value)))


def all_leds(r, g, b):
    if not do_flash:
        return
    r, g, b = int(r), int(g), int(b)
    for i in range(2):
        Board.RGB.setPixelColor(i, Board.PixelColor(r, g, b))
        s.setPixelColor(i, Board.PixelColor(r, g, b))
    Board.RGB.show()
    s.show()


def ledbeepfor(rgbv, dton, dtoff=0.0):
    if __stop:
        return
    buzzer(1)
    all_leds(*rgbv)
    if waitif(dton):
        return
    buzzer(0)
    all_leds(*rgb['black'])
    if waitif(dtoff):
        return


def mean(li):
    return sum(li) / len(li)


def median(li):
    return sorted(li)[int(len(li) / 2)]


def voltage_detection():
    try:
        waitif(0.1)
        v = Board.getBattery() / 1000.0
        if 0 < v < 16:
            return v
    except Exception as e:
        print('Error', e)


def voltage_color(voltage: float):
    if voltage < 3.5:
        return rgb['red']
    if 3.5 <= voltage < 3.7:
        return rgb['orange']
    if 3.7 <= voltage < 3.9:
        return rgb['yellow']
    if 3.9 <= voltage < 4.05:
        return rgb['lime']
    if voltage >= 3.9:
        return rgb['green']
    return (100, 0, 100)


def voltage_goodness(voltage: float):
    if voltage is None:
        return "[░░░░░] ERROR"
    if 0.8 <= voltage < 3.3:
        return "[    ]  VERY LOW CHARGE IMMEDIATELY"
    if 3.3 <= voltage < 3.5:
        return "[■   ] Low "
    if 3.5 <= voltage < 3.7:
        return "[■■  ] Normal"
    if 3.7 <= voltage < 3.9:
        return "[■■■ ] Normal"
    if 3.9 <= voltage < 4.05:
        return "[■■■ ] Near Full"
    if 4.05 <= voltage < 5:
        return "[■■■■] Full"
    return "[░░░░░] ERROR"


def decimal_split(x: float, precision=1, mode='digit'):
    x = round(x, precision)
    s = f"{x:.{precision}f}"
    a, b = s.split('.')
    if mode == 'digit':
        return tuple(tuple(int(x) for x in part) for part in (a, b))
    if mode == 'whole':
        return int(a), int(b)


def beepn(n, color):
    if n == 0:
        ledbeepfor(color, .09, .2)
        return
    for _ in range(n):
        ledbeepfor(color, .3, .2)


def measure_voltage(n: int = 1):
    measurements = [voltage_detection() for _ in range(n)]
    measurements = [x for x in measurements if x is not None]
    if not measurements:
        return None, None
    voltage = median(measurements)
    cell_voltage = voltage / 2
    return cell_voltage, measurements


def loop():
    v, _ = measure_voltage(5)
    color = voltage_color(v)
    a, b = decimal_split(v, precision=1, mode='whole')
    print(f"{a}.{b}\t{v:.3f}\t{color}")
    beepn(a, color)
    if waitif(1):
        return
    beepn(b, color)
    if waitif(2.2):
        return


def stop():
    global __stop
    __stop = True
    buzzer(0)
    all_leds(*rgb['black'])
    print("battchk.py will stop soon.")
    if button_listen:
        print("Waiting for all buttons to be released...")
        return
        # while True:  # let the listener handle the exit.
        #     if not any(btn_check()):
        #         break
        #     time.sleep(0.1)
    if buttonman:
        buttonman.TaskManager.unregister()
    print("Exiting battchk.py")
    sys.exit()  # exit the python script immediately


def main():
    if do_beeps:
        Board.setBuzzer(0)  # initialize buzzer
    if do_flash:
        s.setRGBMode(0)
    for _ in range(n):
        if not __stop:
            loop()
    if not n:  # if n == 0 or None
        cell, measurements = measure_voltage(10)
        print(measurements)
        print(f"Status:\t\t{voltage_goodness(cell)}")
        print(f"Cell Voltage:\t{cell:.3f}")
        return
    while __stop and button_listen and KDN in button_states:
        time.sleep(spin_period)  # trap if waiting for buttons to be unpressed...


def _watch():
    cell, _ = measure_voltage(2)
    if cell and cell < BAD_CELL_VOLTAGE:
        measurements = []
        for _ in range(20):
            total = voltage_detection()
            if total is not None:
                measurements.append(total / 2)
            time.sleep(0.49)
        if measurements and max(measurements) < BAD_CELL_VOLTAGE:
            print("Battery voltage is low. Stopping all registered processes.")
            buttonman.TaskManager().close_all_registered()
            buttonman.stop_board()
            main()
            time.sleep(120)
    time.sleep(60)


def watch():
    # for constantly checking the voltage
    while not __stop:
        try:
            _watch()
        except Exception as err:
            print(err)


def btn_check():
    global button_states
    button_states = [GPIO.input(KEY1_PIN), GPIO.input(KEY2_PIN)]
    return KDN in button_states


def btn_handler(channel, state):
    global button_states
    button_i = 0 if channel == KEY1_PIN else 1
    button_states[button_i] = state
    print(button_states)
    if KDN in button_states:
        stop()
        print("made it out of handler")
    elif __stop:  # both buttons are unpressed
        print("Both buttons unpressed now.")
        print("Exiting battchk.py via handler")
        if buttonman:
            buttonman.TaskManager.unregister()
        sys.exit()  # exit the python script immediately


def setup_buttons():
    global key1_debouncer, key2_debouncer
    print("Adding listeners to stop battchk.py")
    GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    key1_debouncer = buttonman.ButtonDebouncer(KEY1_PIN, btn_handler, bouncetime=30)
    key2_debouncer = buttonman.ButtonDebouncer(KEY2_PIN, btn_handler, bouncetime=30)
    key1_debouncer.start()
    key2_debouncer.start()
    GPIO.add_event_detect(KEY1_PIN, GPIO.BOTH, callback=key1_debouncer)
    GPIO.add_event_detect(KEY2_PIN, GPIO.BOTH, callback=key2_debouncer)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--__listen_button_exit', action='store_true')
    parser.add_argument('--silent', action='store_true')
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('-s', '--stealth', action='store_true')
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('-n', type=int, default=None)
    args = parser.parse_args()

    if not args.watch:
        buttonman.TaskManager.register_stoppable()

    if args.silent or args.quiet:
        do_beeps = False

    if args.n is not None:
        n = args.n

    if args.stealth:
        if args.n is None:
            n = 0
        do_beeps = False
        do_flash = False

    signal.signal(signal.SIGINT, lambda s, h: stop())

    if args.watch:
        watch()
    else:
        button_listen = args.__listen_button_exit
        if button_listen and buttonman:
            setup_buttons()
        elif button_listen:
            raise ImportError("Requested to run in button stop mode, but buttonman module couldn't be imported! Exiting...")
        main()
