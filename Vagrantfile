# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure("2") do |config|
  # Use Ubuntu 16.04 box
  config.vm.box = "bento/ubuntu-18.04"

  config.vm.hostname = "mrover-devbox"

  # Map all ports 8000-8020 for development work
  (8000...8021).each do |p| 
    config.vm.network "forwarded_port", guest: p, host: p
  end

  # Set up rsynced folder to automatically sync
  config.vm.synced_folder ".", "/vagrant"

  # Provision using the devbox playbook
  config.vm.provision "ansible_local" do |ansible|
      ansible.playbook = "ansible/vagrant_devbox.yml"
  end
end
