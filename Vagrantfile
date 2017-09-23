# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure("2") do |config|
  # Use Ubuntu 16.04 box
  config.vm.box = "bento/ubuntu-16.04"

  config.vm.hostname = "mrover-devbox"

  # Map webpack devserver port and static resources port
  config.vm.network "forwarded_port", guest: 8000, host: 8000
  # Map base_station_bridge port
  config.vm.network "forwarded_port", guest: 8001, host: 8001

  # Set up rsynced folder to automatically sync
  config.vm.synced_folder ".", "/vagrant", type: "rsync",
	rsync__exclude: [".git/", "build/", "jarvis_files/env/"]

  # Set up gatling-rsync
  if Vagrant.has_plugin?("vagrant-gatling-rsync")
    config.gatling.latency = 1.0
    config.gatling.time_format = "%H:%M:%S"
    config.gatling.rsync_on_startup = true
  end

  # Provision using the devbox playbook
  config.vm.provision "ansible_local" do |ansible|
      ansible.playbook = "ansible/vagrant_devbox.yml"
  end
end
