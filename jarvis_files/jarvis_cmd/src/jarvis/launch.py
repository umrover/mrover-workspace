import time
import subprocess
import errno
import os

#########################
#
# Function that parses launch command
#
########################
def launch_dir(ctx, package, ssh, opts):
    if package == "percep":
        launch_perception(ssh, opts)
    if package == "nav":
        launch_navigation(ssh, opts)
    if package == "loc":
        launch_localization(ssh, opts)
    if package == "auton":
        launch_auton(ssh)
    if package == "odrive":
        launch_odrive(ssh)

#########################
#
# Functions that build and execute auton subteam code
#
########################
drive_ip = "10.1.0.1"
auton_ip = "10.1.0.2"

def launch_perception(ssh, opts=""):
    wid = launch_terminal()
    build_and_run_package(wid, 'jetson/percep', ssh=ssh, ip=auton_ip)
    new_tab(wid)
    exec_cmd(wid, "./jarvis exec lcm_tools_echo Obstacle /obstacle", ssh=ssh, ip=auton_ip)

def launch_navigation(ssh, opts=""):
    wid = launch_terminal()
    build_and_run_package(wid, 'jetson/nav', ssh=ssh, ip=auton_ip)

def launch_localization(ssh, opts=""):
    wid = launch_terminal()
    exec_cmd(wid, "./jarvis build jetson/gps; sudo ./jarvis exec jetson/gps", ssh=ssh, ip=auton_ip)
    # unable to enter sudo password because have no idea when the build will end
    new_tab(wid)
    build_and_run_package(wid, 'jetson/filter', ssh=ssh, ip=auton_ip)

def launch_odrive(ssh, opts=""):
    wid = launch_terminal()
    build_and_run_package(wid, 'jetson/odrive_bridge', ssh, ip=drive_ip, exec_opts="0")
    new_tab(wid)
    build_and_run_package(wid, 'jetson/odrive_bridge', ssh, ip=drive_ip, exec_opts="1")
    new_tab(wid)
    build_and_run_package(wid, 'jetson/odrive_bridge', ssh, ip=drive_ip, exec_opts="2")

def launch_auton(ssh):
    launch_localization(ssh)
    launch_navigation(ssh)
    launch_perception(ssh)

#########################
#
# Functions for interacting with the terminal
#
########################
def build_and_run_package(wid:str, package:str, ssh:str="", ip:str="", build_opts:str="", exec_opts:str=""):
    jetson_percep = 'jetson/percep'
    
    build = "./jarvis build " + package + " " + build_opts
    exec = "./jarvis exec " + package + " " + exec_opts

    # Combine build and exec command into one, that way
    # the build finishes before the exec
    exec_cmd(wid, build + "; " + exec, ssh, ip)

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

def exec_cmd(wid:str, command:str, ssh:bool=False, ip:str=""):
    if ssh :
        exec_ssh(wid, ip)

    focus = gen_focus(wid)
    cmd = "xdotool key type \'" + command + "\'"
    enter = "xdotool key KP_Enter"
  
    commands = focus + "; " + cmd + "; " + enter

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
    