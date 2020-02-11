#!/usr/bin/expect
#
#1. ssh into a server with username and password
#2. modify /tmp/system.cfg
#- radio.1.chanbw=1
#- radio.1.freq=2
#- radio.1.txpower=3

#add ability to change wireless mode from station to access point and vice versa

#add ability to change lock to ap when in station mode

set USER [lindex $argv 0]
set PASSWORD [lindex $argv 1]
set RADIO [lindex $argv 2]

set timeout 20

if { $argc != 7 && $argc != 8 } {
    puts "USAGE: ./radio_setup.tcl \[username\] \[password\] \[radio ip\] \[chanbw value\] \[channels value\] \[txpower value\] \[wireless mode\] \[access point radio ip \(optional\)\]"
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

set MODESTRING "radio.1.mode="
set MODEVALUE [lindex $argv 6]

#TODO if modevalue is managed or master, leave it and don't cause errors

if {$MODEVALUE == "station"} { 
    set MODEVALUE "managed"
    set LOCKRADIO [lindex $argv 7]
}
if {$MODEVALUE == "accesspoint"} {
    set MODEVALUE "master"
}
append MODE $MODESTRING $MODEVALUE

spawn ssh $USER@$RADIO

expect "*?assword" { send "[lindex $PASSWORD]\r" }


expect "# " { send "\$(sed -i -e \"s/radio.1.chanbw=\[0-9\]*/[lindex $CHANBW]/g\" /tmp/system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/radio.1.freq=\[0-9\]*/[lindex $CHANNELS]/g\" /tmp/system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/radio.1.txpower=-*\[0-9\]*/[lindex $TXPOWER]/g\" /tmp/system.cfg)\r" }
expect "# " { send "\$(sed -i -e \"s/radio.1.mode=.*/[lindex $MODE]/g\" /tmp/system.cfg)\r" }

if {$MODEVALUE == "managed"} {

    set MACADDRESS "wireless.1.ap="

    if {$LOCKRADIO == "10.9.0.1"} {
        append MACADDRESS "68:72:51:80:26:A2"
    } 
    if {$LOCKRADIO == "10.9.0.2"} {
        append MACADDRESS "68:72:51:80:27:45"
    } 
    if {$LOCKRADIO == "10.9.0.3"} {
        append MACADDRESS "68:72:51:8A:29:27"
    } 
    if {$LOCKRADIO == "10.9.0.5"} {
        append MACADDRESS "68:72:51:8A:29:E8"
    }

    expect "# " { send "\$(sed -i -e \"s/wireless.1.ap=.*/[lindex $MACADDRESS]/g\" /tmp/system.cfg)\r" }
}

expect "# " { send "save && reboot\r" }
expect "# " { send "exit\r" }

interact
