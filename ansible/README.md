# Ansible System Configurations

In this directory, we include a set of Ansible configurations for creating a
development environment and setting up the real rover systems.

To use one of these Ansible playbooks on Ubuntu 16.04, make sure Ansible is
installed:

```sh
$ sudo apt install ansible
```

Then run the playbook of your choice.
```sh
$ ansible-playbook -K -i "localhost," -c local <playbook>.yml
```
If you need to define extra variables add --extra-vars "<variable definition>"
example
```
$ ansible-playbook -K -i "localhost," -c local <playbook>.yml --extra-vars "num = 2"
```

## Development Box

The Ansible machine configuration is in `devbox.yml`. It may be used to set up
a development environment on any Ubuntu system.

## Vagrant Box

The Vagrant setup configuration is in `vagrant_devbox.yml`. It should *not* be
used outside the Vagrant box.

## Production MRover systems

The configuration in `jetson.yml` may be used to set up the main on-board
computer on an actual rover. This script makes several assumptions about the
system architecture and is not guaranteed to work outside of the Jetson
TX-series SoC boards.

`jetson.yml` executes roles that configure `systemd` services, `udev` rules,
and environment files for the jetson components. It is intended that running
this playbook will configure the Jetson TX-series board we are using such that
all necessary processes will run on startup.

# System Components

## USB Dev Rules

Our system's custom usb dev rules are located in roles/jetson_service/files/99-usb-serial.rules. \
To add a new one plug in your usb device and type: \
```sudo lsusb -v | grep 'idVendor\|idProduct\|iProduct\|iSerial'``` \
Find your device and it's idVendor, idProudct, iProduct and iSerial numbers/tags (it is ok if your device doesn't have all that information). Go into the 99-usb-serial.rules file and add a new line in this format: \
```SUBSYSTEM=="< your subsystem, normally tty for sub >", ATTRS{idVendor}=="< idVendor >", ATTRS{idProduct}=="< idProduct >", ATTRS{iProduct}=="< iProduct >", ATTRS{serial}=="< serial >", SYMLINK+="< name >" ``` \
The name can be anything. Save the file and re-run this ansible script. If you don't want to re-run it you can edit the ```/etc/udev/rules.d/99-usb-serial.rules``` files directly on the jetson but ideally it should be done this way. \
Save the file and reboot, you now should be able to access your serial device as /dev/< name > \
Here are some useful links: \
[First part was useful](https://community.openhab.org/t/how-to-make-symlinks-for-usb-ports-in-linux-extra-java-opts/89615) \
[Further explanation](https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux) \
[Also helpful](https://inegm.medium.com/persistent-names-for-usb-serial-devices-in-linux-dev-ttyusbx-dev-custom-name-fd49b5db9af1) \





