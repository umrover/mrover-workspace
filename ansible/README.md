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
