#!/bin/bash
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout stable-1.6
sudo rm /usr/bin/ethercat
sudo rm /etc/init.d/ethercat
./bootstrap  # to create the configure script
./configure --prefix=/usr/local/etherlab  --disable-8139too --disable-eoe --enable-generic

make all modules
sudo make modules_install install
sudo depmod

source symlink.sh

# * Create a new :code:`udev` rule:
sudo echo 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' > /etc/udev/rules.d/99-EtherCAT.rules

# * Configure the network adapter for EtherCAT:
#   In the configuration file specify the mac address of the network card to be used and its driver
#     MASTER0_DEVICE="ff:ff:ff:ff:ff:ff"  # mac address
#     DEVICE_MODULES="generic"
ifconfig -a | more
sudo nano /usr/local/etherlab/etc/sysconfig/ethercat

echo "Now we can start the EtherCAT master"
sudo /etc/init.d/ethercat start
