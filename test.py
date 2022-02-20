import time
import subprocess
import os
import errno

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

def gen_focus(wid:str):
    return "xdotool windowfocus " + wid


def exec_cmd(wid:str, command:str):
    
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

def ssh(wid:str, ip:str):
    exec_cmd(wid, "ssh mrover@" + ip)
    time.sleep(1)
    exec_cmd(wid, "mrover")
    time.sleep(1)
    exec_cmd(wid, "cd ~/mrover-workspace")

if __name__ == "__main__":
    wid = launch_terminal()
    exec_cmd(wid, "echo alright")
    new_tab(wid)
    exec_cmd(wid, "echo superb")
    new_tab(wid)
    ssh(wid, "mgmii@login-course.engin.umich.edu")

    