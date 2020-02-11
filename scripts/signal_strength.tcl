#!/usr/bin/expect

# A script for getting
# Intended to be called by python program in /vagrant/base_station/radio_update
# To execute the python program, run:
    # jarvis build base_station/radio_update
    # jarvis exec base_station_radio_update

# Command line arguments

set USER [lindex $argv 0]
set PASSWORD [lindex $argv 1]
set RADIO [lindex $argv 2]

# Constants

# Radio connection timeout
set timeout 20
# Directory for storing files
set TMPDIR /tmp 
# Name of temp data file
set DATA_FILE tmp_radio_data.json

# Error if invalid argument count
if { $argc != 3 && $argc != 4 } {
    puts "USAGE: ./signal_strength.tcl \[username\] \[password\] \[station radio ip\] (optional: temp-file directory)"
    exit 1
}

# If temp-file directory argument is specified
if { $argv == 4 } {

    # If specified path is actually a directory, set TMPDIR to the path
    if { isdirectory $argv } {
        set TMPDIR argv
    }
    # If specified path isn't actually a directory, error and quit
    else {
        puts "Specified temp-file directory is not a directory"
        exit 1
    }
}

# Create data file if doesn't exist
spawn bash -c "touch $TMPDIR/$DATA_FILE"

# Initiate secure copy of radio data file to base station
spawn bash -c "scp $USER@$RADIO:/tmp/stats/wstalist $TMPDIR/$DATA_FILE"

# Wait for request for password, then send password
expect {
    "*?assword" {
        send "[lindex $PASSWORD]\r"
        
        # Wait for end of scp file to arrive
        expect eof
    }
    timeout {
        # Python script checks for this message
        # If you change this message here, change it in the script as well
        puts "Connection to radio timed out"
        exit 1
    }
}

# Get the first occurence of "signal" in temp data file
spawn bash -c "cat $TMPDIR/$DATA_FILE | grep -m1 signal"

interact
