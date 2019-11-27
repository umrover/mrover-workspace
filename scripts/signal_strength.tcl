#!/usr/bin/expect

set USER [lindex $argv 0]
set PASSWORD [lindex $argv 1]
set RADIO [lindex $argv 2]

if { $argc != 3 } {
    puts "USAGE: ./signal_strength.tcl \[username\] \[password\] \[station radio ip\]"
    exit 1
}

set timeout 60

spawn bash -c "touch ./signal.txt"
spawn bash -c " > ./signal.txt"


while { true } {
    spawn bash -c "cat /tmp/temp_signal.txt | grep -m1 signal >> ./signal.txt"
    spawn bash -c "scp $USER@$RADIO:/tmp/stats/wstalist /tmp/temp_signal.txt"

    sleep 5

    expect {
        "*?assword" {
            send "[lindex $PASSWORD]\r"
        }
    }

    interact
}
