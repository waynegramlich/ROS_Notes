# ROS Notes

Some notes on installing and using ROS (Robot Operating System):

## Overview

The basic steps for bringing up ROS are:

1. Reading up about configuring WiFi networks.
2. Install ROS on a virtual machine running on your workstation.
   * Run robot navigation simulator on your workstation.
3. Install ROS onto a Raspberry Pi 3B or 3B+.
   * Run some sort of "hello world" program.
4. Install a Raspberry Pi camera on to the RasPi.
   * Run fiducial software on your Raspberry Pi.
5. Install Raspberry Pi on Loki.
   * Run keyboard tele-operation on Loki
6. Write first ROS Python Program and run it on Loki
   * Drive robot forward and backward.

Without any further adieu, let's get started.

## Configuring WiFi Networks

Most ROS robots are mobile in that they move around the environment.
What this means is that ROS relies heavily on the standard internet
protocols with a particular emphasis on WiFi wireless networking.
This allows you control, monitor, and debug your ROS robot from the
comfort your development workstation (i.e. a desktop or laptop),
while the ROS mobile robot is moving around on the floor.

For the internet, each computer attached to the network is given an address.
For IPv4 (Internet Protocol version 4), the address is the form A.B.C.D, where
A, B, C, and D each represent an 8-bit byte as a decimal number (e.g.
`192.168.0.123`, `10.234.7.6`, etc.)  Since IP addresses change, the internet
has an additional level indirection called DNS (Domain Name Service) that maps
from a human readable name to the IP address number
(e.g `google.com` => `216.58.195.78`.)

A complication occurs with computers that attach to the network via WiFi.
WiFi typically uses a DHCP (Dynamic Host Configuration Protocol) server to
assign IP addresses to computers that log into a network using WiFi.  When
the DHCP server is configured, it is setup to allocate IP addresses from a
range (e.g. 192.168.0.100 through 192.168.0.150.)  The problem that ensues
is that when your ROS robot logs into a WiFi network, you do not actually
know which IP address the DHCP picked to assign to it.

The solution to not knowing your ROS Robot IP address number, is to use
a set of network technologies that goes by the name `zeroconf`.  What this
does is give each robot and development workstation computer a unique DNS name.
The primary requirement for `zeroconf` is that all computers be connected
to the same LAN (Local Area Network.)

This begs the question of "just what is a local area network?"  The technical
answer is that a local area network is a set of computer/network nodes that
have the same basic internet address (e.g 192.168.0.x, 10.123.x.x, etc.)
In practical terms, if you have a WiFi router, all computers that log in via
WiFi *AND* all computers that are hardwired connected to the LAN ports on the
router are on the same local area network.  (Yes, there are exceptions, but
for most people, the previous statement is correct.)

In order for `zeroconf` to work, each development workstation and robot
*MUST* be given a different host name.  The reason for this is because,
`zeroconf` references each machine by `HOSTNAME.local`, where `HOSTNAME`
is the machine's host name.  If there are two or more machines with the same
`HOSTNAME`, total confusion occurs.

The bottom line is that `zeroconf` is really important and in order for it
to work you must ensure that the host name for all of your robots and development
workstations unique (i.e. different from one another.)  For example, if you
have a robot lab with 10 robots and 20 development workstations, you will
need to have a total of 30 different host names.

## Install ROS on Virtual Machine Running on Your Development Workstation

A development workstation is a desktop or laptop with a 64-bit Intel processor on it.

* VirtualBox Installation:
  * Download [VirtualBox Software](https://www.virtualbox.org/wiki/Downloads) for you workstation.
  * Read [Virtual Box Documentation](https://www.virtualbox.org/wiki/Documentation).
  * Download [Ubiquity Robotics Virtual Machine Image](https://downloads.ubiquityrobotics.com/)
  * Load Ubiquity Robotics Virtual Machine Image into VirtualBox and start it up.
    * Be sure that the virtual machine network is configured in "bridged mode"!!!
  * Login as user `ubuntu` and with a password `ubuntu`:
  * Click the terminal window icon (left column with black rectangle and carret) to open terminal.
  * Become superuser `sudo -s` (type in password of `ubuntu`).
  * From VirtualBox menu bar select `[Devices=>Insert Guest Additions CD Image...]`.
  * Change directory to guest additions directory (`cd /media/ubuntu/VBOXADD*`).
    {Is this correct?}
  * Install guest additions (`sudo ./runasroot.sh`).
    {Is this correct?}

* Do some system administration
  * Make sure you are still logged in as root by running `sudo -s` (type in password of `ubuntu`).
  * Decide on the host name for your virtual machine.  You could use your first name followed
    by the letters `ws` (e.g. `alicews`, `bobws`, etc.)  In the instructions below, replace
    `NEWHOSTNAME` with whatever use selected.  Use all lower case letters.
    * Copy `NEWHOSTNAME` into `/etc/hostname` via `echo NEWHOSTNAME > /etc/hostname`.
    * Use `nano /etc/hosts` to edit `/etc/hosts`.  Replace line with `127.0.1.1  ubuntu`
      with line `127.0.1.1 NEWHOSTNAME NEWHOSTNAME.local`.
  * Decide on your user account name.  In a robotics lab situation, we recommend using
    your first name followed by your last name (e.g. `alicesmith`, `bobliu`, etc.)
    In a non-robotics lab situation, your first name should be good enough.  Always use
    all lower case.
  * Create new user account (in the instructions below `USER` is used; pick something different):
    * `adduser USER`         # Provide password and other information.  Remember the password.
    * `adduser USER sudo`    # Adds you to the `sudo` group that lets you use `sudo` program
    * `adduser USER dailout` # Adds you to the `dailout` group that lets you use a terminal emulator
  * Reboot the virtual machine using `sudo reboot`
  * Login it your new USER account with the password you specified previously.
  * Verify that `sudo` works by typing `sudo ls`.  (Specify your password when prompted.)
  * Update your system by doing the following:
    * `sudo apt update`
    * `sudo apt upgrade`

* Configure your virtual machine to suit you preferences.  For `wayne` this is:
  * Install kubuntu plasma (Wayne perfers KDE; everybody else can skip this step)
    * `sudo add-apt-repository ppa:kubuntu-ppa/backports`
    * `sudo apt install -y kubuntu-desktop`
      * Select `lightdm` for the display manager when prompted
    * Logout and log back in under plasma
    * Set keyboard bindings using systems settings
  * Install your favorite editor (e.g. `sudo apt-get install vim`)

* Create and configure ROS catkin workspace:
  * Run `mkdir -p ~/catkin_ws/src`
  * Clone ROS_NOTES repositiory into that newly created directory

        cd ~/catkin/ws/src
	git clone https://github.com/waynegramlich/ROS_Notes.git

  * Copy `.ros_setup.bash` up to your home directory

        cp ~/catkin/ws/src/ROS_NOTES/.ros_setup.bash ~
  
  * Edit your `~/.bashrc` to have the `source ~/.ros_setup.bash` in it.
  * Run `source ~/.bashrc` to force you current shell to have ROS stuff in it.
  * Run `catkin_make`

* Verify that ROS runs on virtual machine:
  * Install some launch files into your ROS workspace:

        cd ~/catkin_ws/src
	git clone https://github.com/UbiquityRobotics/ubiquity_launches.git

  * Install some missing packages:

        sudo apt-get install ros-kinetic-turtlebot-description
        sudo apt-get install ros-kinetic-robot-state-publisher
        sudo apt-get install ros-kinetic-yocs-velocity-smoother
        sudo apt-get install ros-kinetic-yocs-cmd-vel-mux
        sudo apt-get install ros-kobuki-safety-controller
	cd ~/catkin_ws
	catkin_make

  * Run robot simulator (`./ubiquity_launches/bin/keyboard_navigate`)
    * When RVIZ comes up select `[2D Nav Goal]`.
    * User mouse click mouse in middle of maze and drop an arrow that points in any direction.
    * Watch the robot drive to the destination.
    * Woo Hoo!
    * Use [File=>Close] to kill the two windows.
    * Type Cntrl-C to kill off everything else.

## Install ROS onto a Raspberry Pi 3B or 3B+

* Download ubiquity robotics Raspberry Pi image.
  * Create a `~/downloads` directory (`mkdir -p ~/downloads`).
  * Change to the `~/downloads` directory (`cd ~/downloads`).
  * Download the [Raspberry Pi Image](https://downloads.ubiquityrobotics.com/pi.html)
    with the following command:

        wget https://cdn.ubiquityrobotics.net/2018-06-07-ubiquity-xenial-lxde-raspberry-pi.img.xz

    (Note: this link may change in the future)

  * Run the Gnome Disk Utility (`gnome-disks`).
  * Plug a USB to SD/micro-SD device into free USB port on your workstation.  Make sure you
    use one of the older USB to SD/micro-SD devices, they work better.
  * On the VirtualBox menu bar, select `[Devices=>USB=>XXX]` where XXX coresponds to the
    device you just plugged in.  XXX will be something like `Generic Storage Device`.
  * The storage device should show up in the disk utility as something like `VBOX HARDDISK`.
  * Using the settings icon in the upper right corner of the disk utility (i.e. 3 short
    horizontal lines), `select [# => Restore Disk Image]`.
  * Use the file chooser to select
    `~/downloads/2018-06-02-ubiquity-xenial-lxde-raspberry-pi.img.xz` and click `[Open]`
    followed by `Start Restoring...`.

* Boot The Raspberry Pi image.
  * Boot the Raspberry Pi:
    * The most reliable way to boot the image is to find an HDMI cable, a USB mouse,
      a USB keyboard and plug them into the Raspberry Pi.
    * Plug in the micro-SD card.
    * Apply power to the Raspberry Pi.
    * After the image boots login as user `ubuntu` and password `ubuntu`.
    * Open a terminal window. 

* Do the same system administration as you did for you workstation.  Make sure you
  select a NEWHOSTNAME that is different from you workstation.  You should use the
  same USER account.  Create the catkin workspace, but do not download either the
  `ROS_Notes` or `ubiquity_launches` packages.  Do not try to run the robot simulator.

* Run `pifi` to configure your WiFi...

## Install Raspberry Pi camera onto RasPi

## Install Raspberry Pi on Loki

## Install Fiducials

Read up about [fiducials](https://github.com/UbiquityRobotics/fiducials).

Install some packages:

        sudo apt-get install ros-kinetic-fiducials
        (cd ~/catkin_ws ; catkin_make)

Get some 8.5" x 11" adhesive labels from Amazon.  We want these to be removable
labels.  It is usually cheaper to buy a couple of boxes and try them out to figure
out how removable they are.



## Write first ROS Python Program and run it on Loki

* Install AVR/Arduino tool chain on the workstation:

        sudo apt-get install arduino
        sudo apt-get install binutils gcc-avr avr-libc uisp avrdude flex byacc bison

* Install Arduino, Arduino-Makefile, and bus_loki repos into your workstation:

        cd ~/catkin_ws/src
	git clone https://github.com/UbiquityRobotics/Arduino.git
	git clone https://github.com/UbiquityRobotics/Arduino-Makefile.git
	git clonehttps://github.com/UbiquityRobotics/bus_loki.git

* Build the Loki firmware.

        cd bus_loki/ref_f
	make clean
	make

* Plug 3.3V USB DFRobot USB to serial connector into correct connector.

* Type `make upload` to force the firmware into the Loki.

LocalWords:  IP HOSTNAME
