# ROS Notes

These are some notes taken during the process of figuring out
how to use ROS (Robot Operating System).  These notes are not
authoritative.

These notes are written in a style where we are the author(s)
and you are the reader(s).  The resulting text is a little
bit less ponderous.

## How To Request Changes to This Document

Before digging into ROS, this is very brief section on
how to request changes to this document.

These notes are written in
  [markdown](http://daringfireball.net/projects/markdown/)
and stored as the file named `README.md` in the following
[github repository](https://github.com/waynegramlich/ROS_Notes).

If you would like to make suggest any corrections, additions,
or questions, please feel free to:

* create your own account on github.com,
* clone this repository,
* make your corrections, and
* generate your a pull request.

The preferred format for asking a question is:

         <BlockQuote>
         *{ your_name: your question ... }*
         <BlockQuote>


## ROS Concepts

Usually, you are directed to start your exploration of ROS
at the
  [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) .
However, we recommend that you start at the
  [ROS/Concepts](http://wiki.ros.org/ROS/Concepts)
wiki page.  The meat of the ROS concepts is in the section
entitled "ROS Computational Graph Level".

ROS is really a meta operating system that is layered on
top of a more traditional operating system.  As of early
2015, the primary operating system is Linux, although there
has been some effort to port ROS to run on top of MacOS
and Windows.

ROS also heavily uses Debian packages.  Thus, it most easily
runs on top of Linux distributions that use Debian packages.
Probably the most popular Debian based Linux distribution
is Ubuntu.

ROS decomposes software into a confederation of computational
processes called *nodes*.  The confederation of nodes use
strongly typed message passing to communicate with one another.
There are two styles of message passing:

* *ROS Topic*: A ROS topic is basically a broadcast mechanism
  where one one or more ROS nodes can broadcast strongly
  typed messages.  There can be more than one broadcaster
  and there can be more than one listener.  Given the broadcast
  metaphor, a topic corresponds to a named radio channel (
  e.g. KFOX, KITS, KRAP, etc.)

* *ROS Service*: A ROS service is a more traditional remote
  procedure call.  It looks like there can be multiple callers,
  but only one callee.

In addition, there seems to be a higher level strategy
called a *ROS Action*, which appears to be a goal oriented
protocol.  At this time, we do not know enough about ROS Actions
to make an intelligent comment about them.

There are some additional observations about the ROS
computational model:

* *Language Agnostic*: Each node can be written in any
  programming language that supports the message passing.
  At this time, the two primary supported languages are
  C++ and Python.  Other languages can be added by writing
  a library that supports the ROS message passing protocols.
  This is a really nice characteristic of ROS.

* *Not Hard Real Time*: The overall computation model does
  not provide any firm hard real time guarantees.  Individual
  nodes can be programmed using a hard real time extensions.
  For example, under Linux there are two hard real time
  extensions --
    [RTLinux](http://en.wikipedia.org/wiki/RTLinux)
  and
    [RTAI](http://en.wikipedia.org/wiki/RTAI) .
  (Apparently, the
    [PR2](http://wiki.ros.org/Robots/PR2)
  robot uses one of these Linux real time extensions for some
  of their nodes.  However, there is no requirement from ROS
  that either of these real-time extensions be used.

* *Non-deterministic*: The underlying operating system can
   schedule nodes for execution in an somewhat arbitrary order.
  Thus, the order in which messages are delivered between
  nodes is non-deterministic as well.  Obviously, there
  is a risk that node scheduling issues will occasionally
  cause the robot to misbehave and/or crash.  However,
  it appears that ROS basically works, so the non-determinism
  must not be too critical of an issue.

* *Debugging*: While classic debuggers can be used for an
  individual node (e.g. *gdb* for C++ or *idle* for Python),
  stopping a node at a breakpoint may cause the entire
  robot to get into a really strange state.  (This is a
  classic problem with real time systems and is not a
  ROS specific issue.)  Instead, tracing and logging
  seems to be the preferred debugging strategy.  The
  ability to record the message traffic between nodes
  in ROS *bags* is a powerful tool.

* *Composition/Decomposition*: By using a node architecture
  with well defined messages between the nodes, it is possible
  to swap out a node with a different one as long as the
  message interfaces are adhered to.  This is a powerful
  benefit of the ROS node model.  In addition, sometimes
  some functionality that can easily be done in a single
  node is broken into several nodes, just so the inter-node
  messages can be tracked.  The decomposition also allows
  nodes to be run on different processors both on the
  robot and off the robot.  This is a useful benefit of
  having the messaging system layered on top of the internet
  protocol

* *Visualization Tools*: Given that tracing/logging is an
  important ROS concept, it is equally important to have
  tools to visualize this mass of information.  *rviz* is
  a critical tool for understanding what is going on.

## Installing Linux/Ubuntu

The first Tutorial from the ROS tutorials is about installing
ROS.  Realistically, you need to get Ubuntu/Linux installed
before you even think about installing ROS.  After that
you can install ROS.

First off, what is the difference between Linux and Ubuntu?
Linux is an operating system that runs on the bare hardware.
(Not that it matters, but Linux is a member of a class of
operating systems called Unix.)  Ubuntu is a "Linux Distribution"
and comes with Linux and a gigantic pile of additional software --
compilers, editors, web browsers, word processor, etc.  While
Linux is a single operating system there are many
  [Linux Distributions](http://distrowatch.com/)
available.

ROS has various releases.  At this time, the two primary
ROS releases are *Hydro* and *Indigo*.  It is important
that all machines involved with a robot run the same version.
Each ROS release is targeted to one or more Ubuntu releases.
For example, Hydro runs on Ubuntu 12.04 or 13.04.  Whereas,
Indigo runs on 14.04.

Ubuntu uses a numbering scheme of YY.MM, where YY is the
last two digits of the year, and MM is the month number.
The two month dates are either 04 (April) or 10 (October).
The April releases (i.e. 04) on even years are LTS 
(Long Term Support) releases that supply bug patches
for 5 years, whereas all other releases are called a
"standard release" and only provide bug patches for about
six months.  Thus, 12.04 and 14.04 are LTS releases
release in April of 2012 and 2014 respectively.

ROS supports having multiple releases installed on the
same machine.  The way to switch between releases is to
run the `/opt/ros/`*release_name*`/setup.bash` script.
As a beginner, you will probably only install one release,
so this cool feature is not going to be used by you.

With ROS there are two platforms to be concerned with:

* *Robot Platform*.  The robot platform is that actual
  robot that can move.  This platform can be either an
  x86 or ARM hardware processor architecture.

* *Development Platform*. The development platform is
  where the ROS programmer sits the compose, compile,
  and debug the ROS software.  These days, this platform
  is almost always an x86 hardware processor architecture.

ROS needs to be installed on both platforms.  The ROS
tutorials use exclusively use robot simulators.  Thus,
only a development platform is required.

Ultimately, the ROS development platform needs to be
running Ubuntu.  However, it is possible to run Ubuntu
on top of both MacOS, Windows if you are willing to
`use a virtualization technology such as VMWare, Parallels,
VirtualBox, etc.  VirtualBox is free and runs on both
Windows and MacOS.

There appear to be plenty of resources available for
installing ROS.  The
  [x86 ROS Installation](http://wiki.ros.org/ROS/Installation)
wiki page is a good place to start.

For installing ROS on an ARM hardware architecture, the
  [ARM ROS Installation](http://wiki.ros.org/indigo/Installation/UbuntuARM)
wiki page is where you should go.  You do not need to do
this until you have an actual physical robot that is using
the ARM hardware architecture.

When you do get a physical robot, you are going to have to
set up a wireless internet connection between the development
platform and the robot platform.  Section 4.12 of
  [Ros By Example Hydro - Volume 1](http://www.lulu.com/shop/search.ep?contributorId=1109044)
by Patrick Goebel discusses how to set up ROS to work over
the internet.  Alas, missing from this discussion, is an
explanation of how to configure the Ubuntu running on the
robot to
  [automatically connect](http://thenewbieblog.wordpress.com/2012/05/01/wifi-hotspot-setup-on-ubuntu/)
to a wireless access point.

## ROS Introductory Tutorials

Pretty much every goes to the
  [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
to start learning ROS.  The tutorials are very command line
oriented.  If you are not comfortable with the Unix command
line, you should work through a
  [UNIX Beginner Tutorial](http://www.ee.surrey.ac.uk/Teaching/Unix/)
to familiarize yourself with the Unix command line.

The first tutorial concerns installing ROS.  Early on you should
install the following two lines in your `~/.bashrc` file:

        $ source /opt/ros/RELEASE/setup.bash
        $ source ~/catkin_ws/devel/setup.bash

in your `~/.bashrc` file, where `RELEASE` is either indigo (most likely)
or hydro (less likely.)  While the first line is mentioned in the
  [ROS Installation Tutororial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
the second line is not.  You really want the second line in your
`~/.bashrc` file, since it will prevent all sorts of weird and
incomprehensible ROS error messages.

The Tutorials are very command line oriented.  In addition, the
tutorials keep asking you to open a new terminal windows to type
commands into.  Yikes, this coats your screen a many windows with
are hard to manage and keep track of.
We must confess, given that ROS has the concept of launch files,
we do not understand why each tutorial does not have its own
`.launch` file, to fire up all of the required ROS nodes.
This would eliminate the plethora of terminal windows.

Until the tutorials provide individual `.launch` files
(and they may never do so), we strongly recommend that you
learn Unix Processes and Jobs. as described
  [Section 5](http://www.ee.surrey.ac.uk/Teaching/Unix/unix5.html)
of the Unix beginner tutorial.  Please pay particular attention
to section 5.3 onward.  each time the ROS Tutorial says, run
the following command in a **new terminal**, we recommend that
you reuse the same terminal window, but run the command in
background mode.  Thus,

        $ rosrun turtlesim turtlesim_node

would be replaced with:

        $ rosrun turtlesim turtlesim_node &

Use the `jobs` command to see what programs are running.
Use `fg %N`, where N is the job number listed by `jobs`
to bring a program to the foreground where you can type
Control-C to kill it as directed by the tutorials.

## ROS Tutorial Observations

### Name Spaces

ROS appears to have a number name spaces, where a Names Space
is someplace where you can name things.  So far, we have indentified
the following name spaces:

* *Packages*: `rospack list` appears to list them all.  Packages
  appear to be able to contain more than one executable.  Currently,
  `rosrun package_name TAB TAB` seems to list all executables.
  (There is probably a better way.)

* *Nodes*: `rosnode list` appears to find all active nodes.
  Node names must be unique, but are not hierarchical.

* *Topics*: `rostopic list` lists the current active topics.

* *Services*: `rosservice list` lists the current active services.

* *Parameters*: The parameters name space appears to be hierarchical.
`rosparam list` lists the currenta active parameters.

## Installing Ubuntu on a Beaglebone Black

We abbreviate Beaglebone Black as BBB.

Please perform the follow steps:

1. Acquire a micro-SD card that has at least 2GB of storage.
   You should also acquire a micro SD to regular SD adpater.

2. Visit the following
     [BBB/UBuntu page](http://elinux.org/BeagleBoardUbuntu#BeagleBone.2FBeagleBone_Black)
   and follow the instructions for downloading the image, verifying
   the checksum, and uncompressing the image.  Do not execute the
   `sudo dd if=... of=/dev/sdX` command yet.

3. The `dd` command is a disk to disk copy command.  In order to use it,
   we need to replace `od=/dev/sdX` with the correct value.  This is a bit
   involved.  First, we figure out what is mounted *before* we plug in
   the micro SD card.  Run the follow command first:

        sudo blkid

4. Now plug the micro SD card into your memory card slot.  Rerun the
   `sudo blkid` commands.  You should see a new entry that looks
   something like:

        /dev/mmcblk0p1 UUID="..." TYPE="..."

    It could also look like:

        /dev/sdc0 UDID="..." TYPE="..."

    The device name is broken into a base name and a partition name.
    For `/dev/mmcblk0p1`, the base name is `/dev/mmcblk0` and the
    partition name is `p1`.  For `/dev/sdc0`, the device name is
    `/dev/sdc` and the partition is `0`.

5. We want to ensure that the device is **NOT** mounted.
   Bad things happen if the `dd` command is run on a device
   that is mounted.  Please run the following command:

        mount

   The `mount` command will list all devices that are mount on
   the computer.  If you see the micro SD card device in the list,
   your system has automatically mounted your micro SD card.
   If you see something like:

        /dev/mmcblk0p1 on /media/...

   You need to run the `umount` command as follows:

        umount /dev/mmcblk0p1

   This will unmount the device.  Sometimes the `umount` command
   will come back with an error that says `Device is busy`.
   When this happens, you need to find the
     [process](http://stackoverflow.com/questions/624154/linux-which-process-is-causing-device-busy-when-doing-umount)
   that is using the file and
     [kill](https://www.digitalocean.com/community/tutorials/how-to-use-ps-kill-and-nice-to-manage-processes-in-linux)
   the process.

   Please rerun the `mount` command to make sure the device is **NOT
   MOUNTED**.

6. Finally, it is possible to run the `dd` command shown on the
     [BBB/UBuntu page](http://elinux.org/BeagleBoardUbuntu#BeagleBone.2FBeagleBone_Black).
   Replace the `/dev/sdX` with the device name without a partition
   name (e.g. `/dev/mmcblk0` or `/dev/sdc`).  This will take a number
   minutes.  It will be done when you see the command line prompt
   again.  It is safe to simply remove the micro SD card because
   the system does not have it mounted.

7. Remove the micro SD card from the SD card adapter and insert it
   into the BBB.  The micro SD card slot is on the bottom of the BBB.

8. Acquire a
     [3.3V USB to 6-pin](http://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393)
   cable.  These cable are available from
     [multiple vendors](http://octopart.com/ttl-232r-3v3-ftdi-5416714)
   as different prices.  These cables are kind of a defacto standard and
   they show up all the time.  You might want to buy more than one of them.

   Once you have one please plug the cable into the Debug Serial
   Header (J1) of on the BBB.  The black wire on the cable (pin 1)
   should go to the pin with the white dot on J1.  Plug the other
   end of the cable into a USB socket on your laptop/desktop.

   Please run the following command under Linux:

        minicom

   The `minicom` command will probably find the serial cable.
   Using Control-A follow by the letter 'o', please use the
   arrow keys and keyboard to ensure that the serial port is
   configured to a speed of `115200`, a parity of `None`,
   a data of `8` a stop bits of `1`.  This is called `8-N-1`
   format.  Make sure that both hardware and software flow
   control are set to 'No'.  When you are done with configuration,
   use the `Exit` option to exit to the terminal window.

9. This is the step where the BBB gets powered up.  You are
   going to need to depress and hold the boot button (see below)
   as you power up the BBB.

   There are two places  where the BBB can be powered up from.
   You can either use the "5V" power jack or the USB client connector 
   located on the bottom of the BBB.  The USB client connector
   is located near the large metal Ethernet connector on top.
   The BBB comes with a short cable that plugs into this connector.
   If you decide to use the USB client connector, leave the small
   end disconnected, but plug the large end into your desktop/laptop.
   For the 5V power jack option, you have to find the correct
   power supply (5 Volts) and the correct connector.  Again,
   do not plug it in yet.

   Now find the boot button located on top near the micro SD
   card located on the bottom.  Unfortunately, the boot button
   is *NOT* labeled.  However, the boot button is the only button
   on that side of BBB.  The boot button tells the BBB where
   to find the Linux kernel.  If the boot button is depressed,
   the BBB will look for the Linux kernel in the micro SD socket;
   otherwise, if the boot button is not depressed, it looks for
   the Linux Kernel in one of the on board BBB chips.  You want
   the BBB to use the Linux kernel that you just so laboriously
   installed on the micro SD card.  So, when you apply power to
   BBB, you will be depressing the boot button.

   Now make sure that you can see the window that is running
   minicom.  As soon as power is applied, you will want to
   start looking at this window.  You will want to see
   `Starting kernel ...`

   This is the step where we apply power to the BBB. While holding
   down the boot button, apply power to the BBB.  Some LED's
   will start to light up and hopefully some text will start
   to show up in minicom.  Once `Starting kernel ...` shows
   up you can release the boot button.  After you see a screen
   full of text scroll by, you can release the boot button.

   When you see `arm login:` in minicom, you are ready to
   login.  Please ensure that it says:

        Ubuntu 14.04.1 LTS arm ttyO0

   a few lines earlier.  The import part is `Ubuntu 14.04`,
   whereas the rest could be slightly different.

10. Please log in by typing in `ubuntu` as the user name
    and `temppwd` as the password.  Note that there are
    to `p`'s in `temppwd`.  When you see:

        ubuntu@arm:~$

    you are in.



## More Stuff

In the "Understanding Topics" tutorial, the rtq graphic
shows the topic as "/turtle1/command_velocity" instead
"/turtle1/cmd_vel".

The "Using rqt_console and roslaunch" Tutorial is really
two separate tutorials, one on `rqt_console` and another
on `roslaunch`.  The `roslaunch` discussion is currently
too terse.  It is probably worth reading the
  [ROS roslaunch](http://wiki.ros.org/roslaunch)
wiki page.

Probably what comes next is to go the through the tf
[tf (transforms)](http://wiki.ros.org/tf/Tutorials).

