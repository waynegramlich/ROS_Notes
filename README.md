# ROS Notes

Some notes on installing and using ROS (Robot Operating System):

## Overview

The basic steps for bringing up ROS are:

1. Install ROS on a virtual machine running on your workstation.
   * Run robot navitation simulator on your workstation.
2. Install ROS onto a Raspberry Pi 3B or 3B+.
   * Run some sort of "hello world" program.
3. Install a Raspberry Pi camera on to the RasPi.
   * Run fiducial software on your Raspberry Pi.
4. Install Raspberry Pi on Loki.
   * Run keyboard teleop on Loki
5. Write first ROS Python Program and run it on Loki
   * Drive robot forward and backward.

## Install ROS on Virtual Machine Running on Your Workstation

Note a workstation is a desktop or laptop with a 64-bit Intel processor on it.

* VirtualBox Installation:
  * Download [VirtualBox Software](https://www.virtualbox.org/wiki/Downloads) for you workstation.
  * Read [Virtual Box Documentation](https://www.virtualbox.org/wiki/Documentation).
  * Download [Ubiquity Robotics Virtual Machine Image](https://downloads.ubiquityrobotics.com/)
  * Load Ubiquity Robotics Virtual Machine Image into VirtualBox and start it up.
    * Be sure that the virtual machine network is configured in "bridged mode"!!!
  * Login as user `ubuntu` and with a password `ubuntu`:
  * Click the terminal window icon (left column with black rectangle and carret) to open terminal.
  * Become superuser `sudo -s` (type in password of `ubuntu`).
  * From VirtualBox menu bar select `[Devices=>Install Guest Additions CD Image...]`.
  * Change directory to guest additions directory (`cd /media/ubuntu/VBOXADD*`).
  * Install guest additions (`./runasroot.sh`).

* Do some system administration
  * Make sure you are still logged in as root `sudo -s` (type in password of `ubuntu`).
  * Pick a new host name (in the instructions below `NEWHOSTNAME` used; pick something different).
    * Copy `NEWHOSTNAME` into `/etc/hostname` via `echo NEWHOSTNAME > /etc/hostname`.
    * Use `nano /etc/hosts` to edit `/etc/hosts`.  Replace line with `127.0.1.1  ubuntu`
      with line `127.0.1.1 NEWHOSTNAME NEWHOSTNAME.local`.
  * Create new user account (in the instructions below `USER` is used; pick something different):
    * `adduser USER`        # Provide password and other information.
    * `adduser USER sudo`   # Adds you to the `sudo` group that lets you use `sudo` program
  * Reboot the virtual machine using `sudo reboot`
  * Login it your new USER account with the password you specified previously.
  * Verify that `sudo` works by typing `sudo ls`
  * Update your system:
    * `sudo apt update`
    * `sudo apt upgrade`

* Configure your virtual machine to suit you preferences.  For wayne this is:
  * Install kubuntu plasma (Wayne perfers KDE; everybody else can skip)
    * `sudo add-apt-repository ppa:kubuntu-ppa/backports`
    * `sudo apt install -y kubuntu-desktop`
      * Select `lightdm` for the display manager when prompted
    * Logout and log back in under plasma
    * Set keyboard bindings using systems settings
  * Install your favorite editor (`sudo apt-get install emacs`)

* Configure ROS catkin workspace:
  * Create catkin workspace
    * `mkdir ~/catkin_ws
    * `cd ~/catkin_ws`
    * `catkin_make`
  * Create `~/.ros_setup` to contain:

        cat << EOF > ~/.ros_setup
        # Only `source /opt/ros/kinetic/setup.bash` if we have not already done so.
        # We assume that this script does not change very often:
        if [ -d "/opt/ros/kinetic/bin" ] ; then
            case ":$PATH:" in
            *:/opt/ros/kinetic/bin:*) ;;
            *) source /opt/ros/kinetic/setup.bash ;;
            esac
        fi
        
        # Only `source ~/catkin_ws/devel/setup` if it exists:
        if [ -f ~/catkin_ws/devel/setup.sh ] ; then
            source ~/catkin_ws/devel/setup.bash ;
        fi
        
        # Define some ROS environment variables:
        export ROS_CATKIN_WS=$HOME/catkin_ws
        export ROS_HOSTNAME=`hostname`.local
        export ROSLAUNCH_SSH_UNKNOWN=1
        export ROS_MASTER_URI=http://`hostname`.local:11311
        EOF

  * Edit your `~/.bashrc` to have the `source ~/.ros_setup` in it.
  * Run `source ~/.bashrc` to force you current shell to have ROS stuff in it.

* Verify that ROS runs on virtual machine:
  * Do a sanity check that ROS is available:

        roscore &
        rosnode list
        rostopic list
        rossrv list
        fg %roscore
        [Cntrl-C]

  * Install a couple of packages into your ROS workspace:
    * Enter you catkin workspace (`cd ~/catkin_ws/src`)
    * Install `ros_notes` (`git clone https://github.com/waynegramlich/ROS_Notes.git`)
    * Install `ubiquity_launches` (`git clone https://github.com/waynegramlich/ROS_Notes.git`)
  * Install some missing packages:
    * `sudo apt-get install ros-kinetic-turtlebot-description`
    * `sudo apt-get install ros-kinetic-robot-state-publisher`
    * `sudo apt-get install ros-kinetic-yocs-velocity-smoother`
    * `sudo apt-get install ros-kinetic-yocs-cmd-vel-mux`
    * `sudo apt-get install ros-kobuki-safety-controller`
  * Run robot simulator (`./ubiquity_launches/bin/keyboard_navigate`)
    * When RVIZ comes up select `[2D Nav Goal]`.
    * User mouse click mouse in middle of maze and drop an arrow that points in any direction.
    * Watch the robot drive to the destination.
    * Woo Hoo!
    * Use [File=>Close] to kill the two windows.
    * Type Cntrl-C to kill off everything.

## Install ROS onto a Raspberry Pi 3B or 3B+

* Download ubiquity robotics Raspberry Pi image.
  * Create a `~/downloads` directory (`mkdir -p ~/downloads`).
  * Change to the `~/downloads` directory (`cd ~/downloads`).
  * Download the [Raspberry Pi Image](https://downloads.ubiquityrobotics.com/pi.html)
    with the following command:

    <!--wget https://cdn.ubiquityrobotics.net/2018-01-13-ubiquity-xenial-lxde-raspberry-pi.img.xz-->
        wget https://cdn.ubiquityrobotics.net/2018-06-02-ubiquity-xenial-lxde-raspberry-pi.img.xz

     (Note: this link will be changing soon.)

  * Run the Gnome Disk Utility (`gnome-disks`).
  * Plug a USB to SD/micro-SD device into free USB port on your workstation.  Make sure you
    use one of the older USB to SD/micro-SD devices, the work better.
  * On the VirtualBox menu bar, select `[Devices=>USB=>XXX]` where XXX coresponds to the
    device you just plugged in.  XXX will be something `Generic Storage Device`.
  * The storage device should show up in the disk utility as something like `VBOX HARDDISK`.
  * Using the settings icon in the upper right corner of the disk utility (i.e. 3 short
    horizontal lines), `select [# => Restore Disk Image]`.
  * Use the file chooser to select
    `~/downloads/2018-06-02-ubiquity-xenial-lxde-raspberry-pi.img.xz` and click `[Open]`
    followed by `Start Restoring...`.

* Boot The Raspberry Pi image.
  * Boot the Raspberry Pi:
    * The most reliable way to boot the image is to find and HDMI cable, a USB mouse,
      a USB keyboard and plug them into the Raspberry Pi.
    * Plug in the micro-SD card.
    * Apply power to the Raspberry Pi.
    * After the image boots login as user `ubuntu` and password `ubuntu`.
    * Open a terminal window. 

* Do the same system administration as you did for you workstation.  Make sure you
  select a NEWHOSTNAME that is different from you workstation.  You should use the
  same USER account.  Create the catkin workspace, but do not download either the
  `ROS_Notes` or `ubiquity_launches` packages.  Do not try to run the robot simulator.

* Run `pyfi` to configure your WiFi...

## Install Raspberry Pi camera onto RasPi

## Install Raspberry Pi on Loki

## Write first ROS Python Program and run it on Loki



