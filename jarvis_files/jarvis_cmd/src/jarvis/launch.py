import time
import subprocess
import errno
import os

#########################
#
# Function that parses launch command
#
########################
def launch_dir(ctx, package, ssh, only_run, opts):
    
    # Set LCM Environment Variables
    wid = launch_terminal()
    exec_cmd(wid, "export LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=255'", 
            ssh=ssh, ip=AUTON_IP, close_terminal = True)
    if ssh :
        wid = launch_terminal()
        exec_cmd(wid, "export LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=255'", 
                ssh=ssh, ip=DRIVE_IP, close_terminal = True)
    
    # Execute Launch Commands
    if package == "percep":
        launch_perception(ssh, only_run, opts)
    elif package == "nav":
        launch_navigation(ssh, only_run, opts)
    elif package == "loc":
        launch_localization(ssh, only_run, opts)
    elif package == "auton":
        launch_auton(ssh, only_run)
    elif package == "odrive":
        launch_odrive(ssh, only_run)

#########################
#
# Functions that build and execute auton subteam code
#
########################
DRIVE_IP = "10.1.0.1"
AUTON_IP = "10.1.0.2"

def launch_perception(ssh, only_run, opts=""):
    wid = launch_terminal()
    build_run_package(wid, 'jetson/percep', ssh=ssh, ip=AUTON_IP, only_run=only_run)
    new_tab(wid)
    exec_cmd(wid, "./jarvis exec lcm_tools_echo Obstacle /obstacle", ssh=ssh, ip=AUTON_IP)

def launch_navigation(ssh, only_run, opts=""):
    wid = launch_terminal()
    build_run_package(wid, 'jetson/nav', ssh=ssh, ip=AUTON_IP, only_run=only_run)

def launch_localization(ssh, only_run, opts=""):
    wid = launch_terminal()
    exec_cmd(wid, 'sudo chmod a+rw /dev/tty*', ssh=ssh, ip=AUTON_IP)
    time.sleep(1)
    exec_cmd(wid, "mrover")
    new_tab(wid)
    build_run_package(wid, 'jetson/gps', ssh=ssh, ip=AUTON_IP, only_run=only_run)
    new_tab(wid)
    build_run_package(wid, 'jetson/imu', ssh=ssh, ip=AUTON_IP, only_run=only_run)
    new_tab(wid)
    build_run_package(wid, 'jetson/filter', ssh=ssh, ip=AUTON_IP, only_run=only_run)

def launch_odrive(ssh, only_run, opts=""):
    wid = launch_terminal()
    build_run_package(wid, 'jetson/odrive_bridge', ssh, ip=DRIVE_IP, exec_opts="0", only_run=only_run)
    new_tab(wid)
    build_run_package(wid, 'jetson/odrive_bridge', ssh, ip=DRIVE_IP, exec_opts="1", only_run=only_run)
    new_tab(wid)
    build_run_package(wid, 'jetson/odrive_bridge', ssh, ip=DRIVE_IP, exec_opts="2", only_run=only_run)

def launch_auton(ssh, only_run):
    launch_localization(ssh, only_run)
    launch_navigation(ssh, only_run)
    launch_perception(ssh, only_run)

#########################
#
# Functions for interacting with the terminal
#
########################
def build_run_package(wid:str, package:str, ssh:str="", ip:str="", build_opts:str="", exec_opts:str="", only_run:bool=False):
    cmd_build = "./jarvis build " + package + " " + build_opts + "; "
    if only_run :
        cmd_build = ""
    cmd_exec = "./jarvis exec " + package + " " + exec_opts

    # Combine build and exec command into one, that way
    # the build finishes before the exec
    exec_cmd(wid, cmd_build + cmd_exec, ssh, ip)

def launch_terminal() -> str:
    # Construct a Fifo (Named Pipe) if it doesn't exist 
    path = "/tmp/mrover_pipe"
    try:
        os.mkfifo(path)
    except OSError as oe:
        if oe.errno != errno.EEXIST:
            raise

    # Launch a gnome terminal, which will write its window id to a pipe
    subprocess.Popen(["gnome-terminal -- bash -c  'xdotool selectwindow > " + path + "; $SHELL'"], shell=True)

    # Read window ID from the pipe and close it
    fifo = open(path, "r")
    wid = fifo.readline()
    os.remove(path)
    return str(int(wid))

def exec_cmd(wid:str, command:str, ssh:bool=False, ip:str="", close_terminal:bool=False):
    if ssh :
        exec_ssh(wid, ip)

    focus = gen_focus(wid)
    cmd = "xdotool key type \'" + command + "\'"
    enter = "xdotool key KP_Enter"
    close = ""
    if close_terminal :
        close = "xdotool key ctrl+d"

    commands = focus + "; " + cmd + "; " + enter + "; " + close

    proc = subprocess.Popen('/bin/bash', stdin=subprocess.PIPE, shell=True)
    proc.communicate(commands.encode('utf-8'))

def new_tab(wid):
    focus = gen_focus(wid)
    new_tab_cmd = "xdotool key ctrl+shift+t"

    commands = focus + "; " + new_tab_cmd

    proc = subprocess.Popen('/bin/bash', stdin=subprocess.PIPE, shell=True)
    proc.communicate(commands.encode('utf-8'))

def exec_ssh(wid:str, ip:str):
    exec_cmd(wid, "ssh mrover@" + ip)
    time.sleep(1)
    exec_cmd(wid, "mrover")
    time.sleep(1)
    exec_cmd(wid, "cd ~/mrover-workspace")

#########################
#
# Helper Functions
#
########################
def gen_focus(wid:str):
    return "xdotool windowfocus " + wid
    