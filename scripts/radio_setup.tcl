#!/usr/bin/expect
#
#1. ssh into a server with username and password
#2. modify /tmp/system.cfg
#- radio.1.chanbw=1
#- scan_list.channels=2
#- radio.1.txpower=3



set USER [lindex $argv 0]
set PASSWORD [lindex $argv 1]

if { $argc != 5 } {
    puts "USAGE: ./executable.tcl \[username\] \[password\] \[chanbw value\] \[channels value\] \[txpower value\]"
    exit 1
}

set CHANBWSTRING "radio.1.chanbw="
set CHANBWVALUE [lindex $argv 2]
append CHANBW $CHANBWSTRING $CHANBWVALUE

set CHANNELSSTRING "wireless.1.scan_list.channels="
set CHANNELSVALUE [lindex $argv 3]
append CHANNELS $CHANNELSSTRING $CHANNELSVALUE

set TXPOWERSTRING "radio.1.txpower="
set TXPOWERVALUE [lindex $argv 4]
append TXPOWER $TXPOWERSTRING $TXPOWERVALUE



set timeout 60

spawn ssh $USER@10.9.0.5

expect "*?assword" { send "[lindex $PASSWORD]\r" }

#use sed to find and replace accordingly from the file

expect "# " { send "\$(sed -i -e \"s/radio.1.chanbw=\[0-9\]*/[lindex $CHANBW]/g\" /tmp/temp_system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/wireless.1.scan_list.channels=\[0-9\]*/[lindex $CHANNELS]/g\" /tmp/temp_system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/radio.1.txpower=-*\[0-9\]*/[lindex $TXPOWER]/g\" /tmp/temp_system.cfg)\r" }
expect "# " { send "exit\r" }

interact


