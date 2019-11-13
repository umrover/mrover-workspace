#!/usr/bin/expect
#
#1. ssh into a server with username and password
#2. modify /tmp/system.cfg
#- radio.1.chanbw=1
#- radio.1.freq=2
#- radio.1.txpower=3



set USER [lindex $argv 0]
set PASSWORD [lindex $argv 1]
set RADIO [lindex $argv 2]

if { $argc != 6 } {
    puts "USAGE: ./radio_setup.tcl \[username\] \[password\] \[radio ip\] \[chanbw value\] \[channels value\] \[txpower value\]"
    exit 1
}

set CHANBWSTRING "radio.1.chanbw="
set CHANBWVALUE [lindex $argv 3]
append CHANBW $CHANBWSTRING $CHANBWVALUE

set CHANNELSSTRING "radio.1.freq="
set CHANNELSVALUE [lindex $argv 4]
append CHANNELS $CHANNELSSTRING $CHANNELSVALUE

set TXPOWERSTRING "radio.1.txpower="
set TXPOWERVALUE [lindex $argv 5]
append TXPOWER $TXPOWERSTRING $TXPOWERVALUE

#TODO: let users pass in radio ip

set timeout 60

spawn ssh $USER@$RADIO

expect "*?assword" { send "[lindex $PASSWORD]\r" }

#use sed to find and replace accordingly from the file

expect "# " { send "\$(sed -i -e \"s/radio.1.chanbw=\[0-9\]*/[lindex $CHANBW]/g\" /tmp/system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/radio.1.freq=\[0-9\]*/[lindex $CHANNELS]/g\" /tmp/system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/radio.1.txpower=-*\[0-9\]*/[lindex $TXPOWER]/g\" /tmp/system.cfg)\r" }
expect "# " { send "save && reboot\r" }
expect "# " { send "exit\r" }

interact


