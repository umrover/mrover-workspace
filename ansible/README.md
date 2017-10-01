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

## Development Box

The Ansible machine configuration is in `devbox.yml`. It may be used to set up
a development environment on any Ubuntu system.

## Vagrant Box

The Vagrant setup configuration is in `vagrant_devbox.yml`. It should *not* be
used outside the Vagrant box.
