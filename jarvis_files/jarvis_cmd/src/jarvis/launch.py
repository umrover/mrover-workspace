import configparser
import os
import sys
import shutil
import time
import subprocess

from .build import build_dir, build_deps

# Function that parses launch command
def launch_dir(ctx, d, opts, ssh):
    if d == "percep":
        launch_perception(ctx, opts, ssh)
    if d == "nav":
        launch_navigation(ctx, opts, ssh)
    if d == "loc":
        launch_localization(ctx, opts, ssh)
    if d == "auton":
        launch_auton(ctx, opts, ssh)  
    return

def get_process_id(name):

    child = subprocess.Popen(['pgrep', '-f', name], stdout=subprocess.PIPE, shell=False)
    response = child.communicate()[0]
    return [int(pid) for pid in response.split()]

def wait_for_click():
    pid = get_process_id("xdotool selectwindow")
    while pid:
        pid = get_process_id("xdotool selectwindow")
        time.sleep(.1)
    return 

def genTerminalLaunchCommand(script_address, ssh):
    command = "gnome-terminal -- bash -c './"
    command = command + script_address

    if ssh:
        return command + " ssh; $SHELL'"
    else:
        return command + "; $SHELL'"

# Functions that build and execute auton subteam code
def launch_perception(ctx, opts, ssh):
    percep = 'jetson/percep'
    use_linter = 'True'
    workspace_relative_address = "jarvis_files/jarvis_cmd/launchScripts/percep"
    
    build_dir(ctx, percep, use_linter, opts)

    ctx.run( genTerminalLaunchCommand(workspace_relative_address, ssh) )
    
    wait_for_click()
    return

def launch_navigation(ctx, opts, ssh):
    nav = 'jetson/nav'
    use_linter = 'True'
    workspace_relative_address = "jarvis_files/jarvis_cmd/launchScripts/nav"
    
    build_dir(ctx, nav, use_linter, opts)
    
    ctx.run( genTerminalLaunchCommand(workspace_relative_address, ssh) )

    wait_for_click()
    return

def launch_localization(ctx, opts, ssh):
    gps = 'jetson/gps'
    filter = 'jetson/filter'
    use_linter = 'True'
    workspace_relative_address = "jarvis_files/jarvis_cmd/launchScripts/loc"

    build_dir(ctx, gps, use_linter, opts)
    build_dir(ctx, filter, use_linter, opts)
    
    ctx.run( genTerminalLaunchCommand(workspace_relative_address, ssh) )
    
    wait_for_click()
    return

def launch_auton(ctx, opts, ssh):
    build_deps(ctx)
    
    lcm_echo = "lcm_tools/echo"
    l = 'True'
    build_dir(ctx, lcm_echo, l , opts)

    launch_navigation(ctx, opts, ssh)
    launch_localization(ctx, opts, ssh)
    launch_perception(ctx, opts, ssh)
    return
