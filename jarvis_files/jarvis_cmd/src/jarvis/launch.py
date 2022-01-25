import time
import subprocess

from .build import build_dir, build_deps

# Function that parses launch command
def launch_dir(ctx, package, opts, ssh):
    if package == "percep":
        launch_perception(ctx, opts, ssh)
    if package == "nav":
        launch_navigation(ctx, opts, ssh)
    if package == "loc":
        launch_localization(ctx, opts, ssh)
    if package == "auton":
        launch_auton(ctx, opts, ssh)

def get_process_id(name):

    child = subprocess.Popen(['pgrep', '-f', name], stdout=subprocess.PIPE, shell=False)
    response = child.communicate()[0]
    return [int(pid) for pid in response.split()]

def wait_for_click():
    pid = get_process_id("xdotool selectwindow")
    while pid:
        pid = get_process_id("xdotool selectwindow")
        time.sleep(.1)

def gen_terminal_launch_command(script_name, ssh):
    command = "gnome-terminal -- bash -c './jarvis_files/launch_scripts/"
    command = command + script_name

    if ssh:
        return command + " ssh; $SHELL'"

    return command + "; $SHELL'"

# Functions that build and execute auton subteam code
def launch_perception(ctx, opts, ssh):
    jetson_percep = 'jetson/percep'
    use_linter = True

    build_dir(ctx, jetson_percep, use_linter, opts)

    ctx.run(gen_terminal_launch_command("percep", ssh))

    wait_for_click()

def launch_navigation(ctx, opts, ssh):
    jetson_nav = 'jetson/nav'
    use_linter = True

    build_dir(ctx, jetson_nav, use_linter, opts)

    ctx.run(gen_terminal_launch_command("nav", ssh))

    wait_for_click()

def launch_localization(ctx, opts, ssh):
    jetson_gps = 'jetson/gps'
    jetson_filter = 'jetson/filter'
    jetson_imu = 'jetson/mp9250_bridge'
    use_linter = True

    build_dir(ctx, jetson_gps, use_linter, opts)
    build_dir(ctx, jetson_filter, use_linter, opts)
    build_dir(ctx, jetson_imu, use_linter, opts)

    ctx.run(gen_terminal_launch_command("loc", ssh))

    wait_for_click()

def launch_auton(ctx, opts, ssh):
    build_deps(ctx)

    lcm_echo = "lcm_tools/echo"
    use_linter = True
    build_dir(ctx, lcm_echo, use_linter, opts)

    launch_navigation(ctx, opts, ssh)
    launch_localization(ctx, opts, ssh)
    launch_perception(ctx, opts, ssh)
