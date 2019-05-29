import lcm
from pathlib import Path
from rover_msgs import AutonState


gpio_pin = '397'
autonomy_on = False
gpio_path = Path('/sys/class/gpio/gpio{}'.format(gpio_pin))
export_file = Path('/sys/class/gpio/export')
unexport_file = Path('/sys/class/gpio/unexport')


def export_pin():
    global gpio_pin, export_file, gpio_path
    export_file.write_text(gpio_pin)
    (gpio_path / 'direction').write_text('out')


def unexport_pin():
    global gpio_pin, unexport_file
    try:
        unexport_file.write_text(gpio_pin)
    except OSError:
        pass


def write_gpio(value):
    global gpio_path

    (gpio_path / 'value').write_text(value)


def auton_state_callback(channel, msg):
    global autonomy_on

    auton_msg = AutonState.decode(msg)
    if auton_msg.is_auton and not autonomy_on:
        write_gpio('1')
        autonomy_on = True
    elif not auton_msg.is_auton and autonomy_on:
        write_gpio('0')
        autonomy_on = False


def main():
    unexport_pin()
    export_pin()
    write_gpio('0')

    lc = lcm.LCM()
    lc.subscribe('/auton', auton_state_callback)

    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        unexport_pin()
