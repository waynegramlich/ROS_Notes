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

In addition, there is a book entitled
  ["A Gentle Introduction to ROS"](http://www.cse.sc.edu/~jokane/agitr/agitr-letter.pdf)
by Jason M. O'Kane that provides a smoother more conceptual
intruction to ROS than the ROS tutorials currently do.
This book was found at
[Jason's web site](http://www.cse.sc.edu/~jokane/agitr/).

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

## ROS Parameters

The ROS parameter system seems to evolved over time and
there does not appear to be a coherent explanation of 
how it all fits together.

The basic concepts behind parameters appear to be:

* The [Parameter Server](http://wiki.ros.org/Parameter%20Server).
  is a ROS node that provide a hierachical naming service.

* There is both a
  [Python API](http://wiki.ros.org/rospy/Overview/Parameter%20Server)
  and a
  [C++ API](http://wiki.ros.org/roscpp/Overview/Parameter%20Server)
  that can be used for getting, setting, and deleting parameter values
  in the parameter server.

* The [ROS Launch](http://wiki.ros.org/roslaunch) facility
  provides a mechanism for setting parameter values, prior to
  starting the nodes the constitute a ROS system.

* The [Dynamic Reconfigrure](http://wiki.ros.org/dynamic_reconfigure/Tutorials)
  facility allows for the dynamic is useful to manuipulate parameters
  in real time.

### Parameter Server

The [Parameter Server](http://wiki.ros.org/Parameter%20Server)
seems to use a hybrid naming system that consists of
/NAMESPACE/NODE_NAME/NAME_PATH, where:

* NAMESPACE is the name space that contains the node name.
  It could be the global namespace `/`.

* NODE_NAME is a ROS node name within the namespae.

* NAME_PATH is either a single top level parameter name or
  a nested hierarchical parameter name.

Very frequently, "/NODE_NAME/" is replaced by "~" to provide
a "private parameter".  Thus, "~parameter_name", is equivalant
to "/my_unique_node_name/parameter_name".

In order for a parameter to show up in the parameter server,
some process has to put it there.  The ROS node can do this
explicitly or it can be done via a ROS `.launch` file.

Likewise, for a parameter to be removed from the parameter
server some process has to remove it.  There is no documented
mechanism for cleaning out stale parameters when a node shuts
down.  (Really?)

The [`rosparam`](http://wiki.ros.org/rosparam) command line program
can be used to access and manipulate contents of the ROS parameter
server.

The supported parameter types are bool, int, float, string,
lists, dictionaries, dates, and binary blobs.

### Python Client Interface

The most common usage for getting a parameter is:

        parameter = rospy.get_param("~parameter_name", default_value)

where `default_value` is returned if the "~parameter_name" is
not in the parameter server.  Calling this routine does *NOT*
insert anything into the parameter server.  If it is not
explicitly put int to server, the `rosparam` command can not
see it.  Calling `rospy.get_param` without a default value will
throw a `KeyError` exception the parameter name is not in the
parameter server.

A parameter can be inserted into the parameter server as follows:

        rospy.set_param("~parameter_name", parameter_value)

If the `~parameter_name` is not present, it is created and set to
`parameter_value`; otherwise, the previous value is overwritten
with `parameter_value`.  If you want the parameter to be
removed when the node goes away, you will need to explicitly
do this by calling `rospy.delete_param`.  Below is an overview
of one way to manage this:

        # Get parameters:
        if rospy.has_param("~parameter_name"):
            parameter = rospy.get_param("~parameter_name")
        else:
            parameter = 123
	    rospy.set_param("~parameter_name", parameter)

        # Main node loop:
        while not rospy.is_shutdown():
            # ...

        # Delete the parameter
        rospy.delete_param("~parameter_name")

In the Python API documentation, the following zinger is present:

> NOTE: parameter server methods are not threadsafe,
> so if you are using them from multiple threads,
> you must lock appropriate[ly].

Multi-threading is an issue for dynamic reconfiguration.

### ROS Launch

The [ROS Launch](http://wiki.ros.org/roslaunch) facility
has the ability to set parameters in the parameter server.

Parameters are set in a `.launch` file, using `<param .../>'
and `<rosparam .../>` tags.  The `<param .../>` tag sets a
single parameter in the parameter server.  The `<rosparam .../>`
tag permits the setting of multiple parameters in the the parameter
server.  It can do this using
  [YAML](http://wiki.ros.org/YAML%20Overview)
(YAML Ain't Markup Language) syntax.  In particular, the
`<rosparam .../>` tag has a mechanism for reading in a
file that is in YAML format.

As a random comment, in section 5.3 (Setting Parameters) of th
  [roslaunch/XML](http://wiki.ros.org/roslaunch/XML)
documentation, it states:

> You can also set parameters on the Parameter Server. These
> parameters will be stored on the Parameter Server before any
> nodes are launched.

Conversely, earlier in section 1 (Evaluation Order) it says:

> `roslaunch` evaluates the XML file in a single pass.
> Includes are processed in depth-first traversal order.
> Tags are evaluated serially and the *last setting wins*.
> Thus, if there are multiple settings of a parameter,
> the last value specified for the parameter will be used.

These two statements do not seem agree with one another.
The prudent thing to do is to put all `<param .../>` and
`<rosparam .../>` tags prior to the associated `<node .../>`
tags in the `.launch` file.

Digging under the covers, we discover that when `roslaunch`
encounters a `<node .../>` tag, it starts the node program
using the `rosrun` command.

The `rosrun` command allows the node name specified in the
node code to be overriden from the command line using a
command line argument of the form:

        rosrun PACKAGE EXECUTABLE __name=NEW_NODE_NAME ...

Somehow, the Python line:

        rospy.init_node("NODE_NAME", ...)

will use `NEW_NODE_NAME` from the command line instead of
`NODE_NAME` from code.

ROS `.launch` files can nest throught the `<include .../>` tag.

### Dynamic Reconfigure

### Combining It All

Within the node code, we need to do the following:

* Initialize each parameter using the following code:

        import yaml

        def dynamic_callback(config, level):
            assert isinstance(config, dict)
            
	
	def main():
	    # At startup for each parameter:
	    parameters = {}
            parameters["parameter1"] = # default parameter 1
            parameters["parameter2"] = # default parameter 2
            parameters["parameter3"] = # default parameter 3
            # ...:
            parameters["parameterN"] = # default parameter N

	    # Stuff everything into the parameter server:
            for parameter_name in parameters.keys():
		private_name = "~" + parameter_name
                if rospy.has_param(private_name):
                    parameters[parameter_name] = rospy.get_param(private_name)
                else:
                    rospy.set_param(private_name, parameters[parameter_name])
	
	    while not rospy.is_shutdown():
                #....

            # During shutdown, write out the YAML:
            yaml_out_stream = open("...", "wa")
            out_stream.write(yaml.dump(parameters))
            yaml_out_stream.close()


### ROS `package.xml`

The documentation for [`package.xml`](http://www.ros.org/reps/rep-0127.html)
format is worth reading.

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
     [3.3V USB to 6-pin](http://elinux.org/Beagleboard:BeagleBone_Black_Serial)
   cable.  These cables are available from
     [multiple vendors](http://octopart.com/ttl-232r-3v3-ftdi-5416714)
   at different prices.  These cables are kind of a defacto standard and
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

11. Follow the remaining steps to resize your micro SD disk.
    When you reboot the BBB, it is no longer necessary to
    hold down the boot button.


## Download ROS Indigo to BBB

Please acquire and connect an ethernet cable between the
BBB and your network.  We are assuming that your network
is running DHCP (Dynamic Host Configuration Protocol) so that
the BBB can access other resources out in the global internet.

Ubuntu uses the Debian package manager for downloading
software packages.  The software packages live in package
repositories located on the net.  The information stored
in the `/etc/apt` directory lists all of the locations
to download software from.  We will need to ensure
that the Debian package manager is configured to find
all of the ARM7 ROS packages.

The
  [ROS UbuntuArm](http://wiki.ros.org/indigo/Installation/UbuntuARM)
web page gives instructions for downloading ROS to an
ARM processor like the BBB.  Just ignore section 2.1.
The Linux image has the correct values in `/etc/apt/sources.list`.
If you want to find out more about the files in `/etc/apt`, the 
  [Repositories Command Line](https://help.ubuntu.com/community/Repositories/CommandLine)
web page explains what is going on.  You really do not need
to bother with this.

Just follow the remaining instructions from section 2.2 onward.

If you can run the `roscore` command, ROS has been installed.

## Additional BBB Configuration

We think it is good to create a new user account on the BBB.
This makes it a little easier to use the `ssh` (Secure Shell)
command to communicate between the BBB and your development
desktop/laptop.  If you are logged onto YOUR_USER_NAME
on your development desktop/laptop, we recommand that you create
a new BBB user account named YOUR_USER_NAME.

The `adduser` command will prompt for the information needed
to create your new user account:

        $ sudo adduser YOUR_USER_NAME
        # Provide passwords and other information here.

The `adduser` command does not add you to the appropriate
secondary groups that the `ubuntu` user account has.
We want YOUR_USER_NAME to have the same privileges that
the `ubuntu` account has.  To see what groups `ubuntu`
has, please run the `groups` command:

        $ groups
        ubuntu adm kmem dailout cdrom ...

In order for the YOUR_USER_NAME account to have the same groups,
the following command will do the trick:

        $ for g in `groups` ; do sudo usermod -a -G $g YOUR_USER_NAME ; done

Note that we use accent graves (backward single quotes) instead
of single quotes in the command above.

From your development desktop/laptop, please try

If you want to change the name of the machine from `arm` to something
else (e.g. NEW_HOSTNAME.)  This is done by:

        sudo echo NEW_HOSTNAME > /etc/hostename
        sed s/arm/NEW_HOSTNAME/ < /etc/hosts > /tmp/hosts
        sudo mv /tmp/hosts /etc/hosts

where NEW_HOSTNAME is replaced by your chosen host name.

Other stuff to download:

        sudo apt-get install emacs

## Removing USB0

To remove the /dev/usb0 network device that shows up when
you power the BBB throught the USB connector it is necessary
to edit to files.

* Edit `/etc/network/interfaces`.  Go to the end and comment
  out the following chunck of configuration:

        iface usb0 inet static
            address 192.168.7.2
            netmask 255.255.255.0
            network 192.168.7.0
            gateway 192.168.7.1

  to look like:

        #iface usb0 inet static
        #    address 192.168.7.2
        #    netmask 255.255.255.0
        #    network 192.168.7.0
        #    gateway 192.168.7.1


* Edit `/opt/scripts/boot/am335x_evm.sh`.  Replace:

        if [ -f /etc/udhcpd.conf] ; then

  with

        if [ false && -f /etc/udhcpd.conf] ; then

  Next, replace:

        if [-f /etc/dnsmasq.d/usb0-dhcp ] ; then

  with

        if [ false && -f /etc/dnsmasq.d/usb0-dhcp ] ; then

  Place a `#` at the being of the line:

        usb0_addr=$(ip addr list usb0 ...

  Replace

        if [ ! "x${usb0_addr}" = "x" ] ; then

  else

        if [ false && ! "x${usb0_addr}" = "x" ] ; then

Now reboot the BBB.

## BBB Serial Port

Way back around Linux kernel 3.2, Linus Torvolds complained
loudly that all of the Arm processors out there were causing
too much kernel source code churn.  This triggered a long and
tedidous development of something called "device trees" that
are primarily used by ARM processors.

A great deal of effort went into developing something called
the cape manager for Beaglebone class of processors.  By
Linux kernel 3.8 it worked reasonably well.  Alas, Ubuntu 14.04
uses Linux kernel 3.14 and the cape manager no longer works
on 3.14.  Thus, configuring the pin-outs on the BBB  for 3.14
requires more tedious device tree configuration.

There are two kinds of device tree files -- `.dts` (Device
Tree Source) and `.dtb` (Device Tree Blob) files.  There
is a compilier, called `dtc`, convert between the two
formats.

When the Linux 3.14 kernel boots on the BBB, it processes a
`.dtb` file to configure the machine including all of the
I/O pins.  The `.dtb` file must live in a directory called
`/boot/dtbs/KERNEL`, where KERNEL is `3.14.22-ti-r31` for
Ubuntu 14.04.

It turns out the `bus_beaglebone` PCB needs to connect to the
UART2 pins on BBB.  These pins a located at pins P9-22 (RX) and
P9-21 (TX).  There are two files locad in the `/boot/dtbs/.../`
called `am335x-boneblack.dtb` and `am335x-boneblack-ttyO2.dtb`
respectively.  The former is what is standardly used during
boot up time.  The later, is one that that configures the two
I/O pins to connect to UART2.

It is useful to see how this all works, so we recommend that
you download the `dtc` compiler using the following command:

        sudo apt-get install device-tree-compiler

Now using the `dtc compiler`, please convert the `.dtb` files
back into `.dtc` files:

        cd /boot/dtbs/3.14.22-ti-r31
        dtc -I dtb -O dts -o /tmp/am334x-boneblack-ttyO2.dts am335x-boneblack-ttyO2.dtb
        dtc -I dtb -O dts -o /tmp/am334x-boneblack.dts am335x-boneblack.dtb

Using your favorite source code difference tool, you see the
differences between the `.dts` files.  Since most decent
source differencing tools use a GUI, we recommend that you
copy the files over your desktop/laptop development machine.
Here are some command that will do that when executed on
the desktop/laptop:

        cd /tmp
        scp "arm.local:*.dts" .
        tkdiff *.dts

The first thing you will note is that within the `.dts` file,
the UART's are called `uart1` through `uart6`.  It turns out
that `/dev/ttyO2` (the serial device we are interested in)
is bound to `uart3` in the `.dtb` file.

If you want to generate your own custom `.dbs` file you
can start with `am334x-boneblack.dts` file and use reverse
compiled `.dts` files from the `/boot/dtbs/...` directory
to make changes.  When you are done, you can compile the
`your.dts` file into a `dtb` file as follows:

    sudo dtc -I dts -O dtb -o /boot/dtbs/.../your.dtb your.dts.

A custom `.dtb` is configured to be loaded at boot time by
editing the file `/boot/uboot/uDev.txt` as
  [described](http://elinux.org/Beagleboard:Capes_3.8_to_3.14#Custom_dtb) .
In short, you add a line that says:

        dtb=your.dtb

If you only need to enable /dev/ttyO2, it is acceptable to just
use the existing `.dtb` file:

        dtb=am335x-boneblack-ttyO2.dtb

Download and install `minicom`:

        sudo apt-get install minicom

Short pins 21 and 22 togetehr and run `minicom`.
When you type into minicom, the characters should echo back.

(Drats it still does not work!)

## Configure BBB for Wireless Access



## Random Stuff to be Deleted

The
  [Device Tree Overlays](http://www.armhf.com/beaglebone-black-serial-uart-device-tree-overlays-for-ubuntu-and-debian-wheezy-tty01-tty02-tty04-tty05-dtbo-files/)
web page seems useful.  The comments are pretty interesting as well.

The following command will produce a `.dts` file from a `.dtb` file:

        dtc -I dtb -O dts -o /tmp/ttyO2.dts /boot/dtbs/3.14.22-ti-r31/am335x-boneblack-ttyO2.dtb

An interesting strategy for 

  [simple dtc stuff](https://github.com/RobertCNelson/rscm/blob/master/build.sh)

  [more dtc stuff](http://hipstercircuits.com/adding-beaglebone-cape-support-to-a-kernel-with-device-tree-in-ubuntu/)

There seem to be a bunch of `.dtb` files in `/boot/dtbs/{os_dir}/`.
`.dtb` files appear to be binary.

There is
  [another tutorial](http://xillybus.com/tutorials/device-tree-zynq-1)
that is focused on FPGA's.

Will I have to
  [compile a BBB kernel](http://wiki.beyondlogic.org/index.php?title=BeagleBoneBlack_Building_Kernel)
to get my grubby paws on a `.dts` file?

Some info about
  [UBoot](http://www.twam.info/hardware/beaglebone-black/u-boot-on-beaglebone-black)
is available.

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

## Banana Pro Configuration:

Below are 

* Copy .img file onto micro SD card:

        time sudo dd if=Lubuntu_For_BananaPro_v1412.img of=/dev/mmcblk0 # 12min

  Obviously, `/dev/mmcblk0` may not be the correct value for your
  system.  See the section on the BeagleBone Black (BBB.)

* Update system (with wired Ethernet):

        time sudo apt-get update	# ~1min
        time sudo apt-get upgrade	# ~25min

* Create your user account:

        sudo adduser YOUR_USER_NAME
        # Provide passwords etc.
        for g in `groups` ; do sudo usermod -a -G $g YOUR_USER_NAME ; done
        sudo vi /etc/sodoers
        # Add line "YOUR_USER_NAME ALL=(ALL:ALL) ALL" to file
        # right after "bananapi ALL=(ALL:ALL) ALL)" in file.

* Login as YOUR_USER_NAME:

        sudo su YOUR_USER_NAME

* Install ROS Indigo:

        sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
        sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros trusty main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.namniart.com/repos/namniart.key -O - | sudo apt-key add -
        time sudo apt-get update			# ~1min
        time sudo apt-get install ros-indigo-ros-base	# ~10min
        sudo apt-get install python-rosdep
	rosdep init
        time rosdep update				# ~1min
        echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        sudo apt-get install python-rosinstall		# ~4min

* Configuring WiFi:
  I found the
  [nmcli discssion](http://arstechnica.com/civis/viewtopic.php?t=1163023)
  to be interesting and useful.

  Here is what has been figured out.  There are 4 systems that
  seem to be interacting with one another:

  * *systemd*. This is the new dependency based system initialization
    system.  There is a great deal of controversy about it.  Regardless,
    of the controversy, it is what is used for initialization in the
    kernel that ships with the Banana Pro.

  * *dbus*.  This is a general purpose publish and subscribe messaging
    system that is used a lot by Linux systems, particularly desktop
    GUI's like Gnome and KDE.

  * *wpa_supplicate*.  This a process that seems to manage wifi interfaces.

  * *network_manager*.  This mostly a GUI program for managing network
    connections.  It also has a command line interface.  The network
    manager also manages its "database" of information about connections.

  All four of these systems are enabled and interact with one another.
  While it is tempting to try and work around them, it appears to
  be easier to just work with what is already there.

  The file `/etc/NetworkManger/NetworkManager.conf` is pretty
  short and configures the network manager.  The line:

        plugins=ifupdown,keyfile,ofono

  is what enables key files.  Key files get stored down in
  `/etc/NetworkManager/system-connections`.  There will be one
  file in this directory for each WiFi access point SSID/password
  pair.  Note that these files are owned by `root` and have root
  only access (i.e. `-rw------- root root`.)

  The `nmcli` program is used to set things up:

  * List the avaiable connections:

            sudo nmcli connect list
            # Connection list shows up here

  * Shut down the wired connection (not sure this works):

            sudo nmcli connect down id "Wired connection 1"
            sudo nmcli device disconnect iface eth0

  * List the available access points:

            sudo nmcli device wifi list
            # Access points listed here

  * Now we can connect to the accesss point:

            sudo nmcli device wifi connect SSID password "PASSWORD"

    where SSID is the access point name and PASSWORD is the
    access point password.  This should create a file named
    `/etc/NetworkManager/system-connections/SSID`.  This
    file contains the SSID and the password (in the clear.)

  * Make sure the rest of zeroconf is installed:

            sudo apt-get install libnss-mdns

  * Unplug the wired cable and reboot:

            sudo reboot

* Resize the file system:

  Follow the
  [resize instructions](http://elinux.org/Beagleboard:Expanding_File_System_Partition_On_A_microSD)
  to get the full extent of your micro-SD card.    

* Bring up the serial port `/dev/ttyS2`

  Perform the following steps:

  1. Shut down the Banana Pro.

            sudo halt

  2. Install a shorting block (i.e. a 2-pin jumper) between
     pins 8 and 10.

  3. Power up the Banana Pro.

  4. Log in remotely from the desktop:

            ssh lemaker.local
            # Type in the password

  5. Download `minicom`:

            sudo apt-get install minicom

  6. Fire up `minicom`:

            minicom -D /dev/ttyS2
            # Use [Control-A] followed by the letter 'o' to enter
            # serial port configuration.  Using the arrow keys
            # select "Serial port setup".  Use the letter 'F' to
            # disable hardware flow control.  Use arrow keys to
            # get down to "Exit"

  7. Type characters into `minicom`.  They should echo because of
     the jump strap.

  8. Remove the jumper strap and type characters.  No characters should
     echo.

  7. Exit `minicom`

            # Type [Control-A] followed by the letter 'x' to exit.

* Some note on .fex files:

  Like all ARM processors, configuring the I/O pins is currently
  pretty challenging.  For the Banana Pro they use something called
  a `.fex` file.  All of this stuff lives over at
  [linux-sunxi.org](http://linux-sunxi.org/).

  It turns out that none of this stuff is necessary since the `.fex`
  file that comes with the kernel has `/dev/ttyS2` already enabled.
  However, it was difficult to figure all this stuff out, so some
  "tickler" notes are listed below:

  * The `.fex` file format is
    [documented](http://linux-sunxi.org/Fex_Guide).

  * The `.fex` boards repository is kept over at `github.com`:

            cd SOMEWHERE
            git clone git://github.com/linux-sunxi/sunxi-boards.git

    The Banana Pro `.fex` file is:

            sunxi-boards/sys_config/a20/BananaPro.fex

  * Installing the `.fex` tools:

            cd SOMEWHERE
            sudo apt-get install libusb-1.0-0-dev
            git clone  git://github.com/linux-sunxi/sunxi-tools.git
            cd sunx-tools
            make

  * Directions for getting and replacing
    [script.bin](http://forum.lemaker.org/thread-221-1-1-.html)

## Random Stuff

I found the
    [rosccpp internals](http://wiki.ros.org/roscpp/Internals)
to be useful.  An
    [associated post](https://bcharrow.wordpress.com/2013/06/17/subscribing-to-a-topic-in-roscpp/)
was useful as well.

## Create ROS Package for a git Repository

### Create the ROS package

As per the tutorials, you go to your catkin workspace:

        cd .../catkin_ws

Now move into the `src` directory:

        cd src

Now run `catkin_create_pkg`:

        catkin_create_pkg new_package_name std_msgs rospy roscpp

where `new_package_name` is the name of the new ROS package.

### Create the Git Repository

(This example uses `github.com` as the repository.  Obviously,
different repository vendors have a different user interface.
Furthermore, most repository vendors change the user interface
many times a year, so the directions below are kind of a hint
for what needs to be done, rather than an exact recipe.)

Using a browser, visit `https://github.com/your_github_account`
where `your_github_account` is your repository account name.

Using the browser, click on the `[Repositories]` tab, followed
by clicking on the `[New]` tab.

Now fill int the `Repository name` field with exactly the
name of the catkin package you created previously (i.e.
`new_package_name`.)  Fill in the `Description` field as well.
Select `(x) Public` and leave
`[ ] Initialize this repository with a README` unchecked.
Finally, click on `[Create repository]`.

## Now we go back and connect the catkin package to `git`.

In `.../catkin_ws/src/new_package_name`, do the following:

1. Create a `README.md` file:

        echo "# new_package_name" >> README.md

2. Get the local `git` repository created:

        git init

3. Add files to repository:

        git add CMakeLists.txt README.md package.md

4. Commit the changes:

        git commit -m "Initial commit."

5. Connect the local repository to the remote repository:

        git remote add origin https://github.com/account_name/new_package_name.git

   where `account_name` is your `github.com` account name and
  `new_package_name` is the new package name.

6. Shove the files up to the remote repository:

        git push -u origin master


All done.

## ROS on Raspberry Pi 2

We are going to shorten Raspberry Pi 2 down to RasPi2
just to save a little typing.

As of 15Apr2015, a version of Ubuntu 14.04 LTS is available
for the RasPi2 from the
  [Raspberry Pi 2 Ubuntu Page](https://wiki.ubuntu.com/ARM/RaspberryPi).
More specically, download the image and unpack it:

        cd ...	# Decide where you are going to download to.
        wget http://www.finnie.org/software/raspberrypi/2015-04-06-ubuntu-trusty.zip
	unzip unzip 2015-04-06-ubuntu-trusty.zip

Follow the directions:

        sudo apt-get install bmap-tools
        sudo bmaptool copy --bmap 2015-04-06-ubuntu-trusty.bmap 2015-04-06-ubuntu-trusty.img /dev/mmcblk0
        sync

Note that the date `2015-04-06` may have to be changed to some
other date you have downloaded a newer release.  The `/dev/mmcblk0`
may need to be a different device; see the section on find
your micro-SD card device.

Next, you need to do the following:

* Remove the micro-SD card from your laptop/desktiop.

* Plug the micro-SD card into your RasPi2.

* Connect an HDMI cable between RasPi2 and an HDMI display.

* Plug a USB keyboard into one of the USB ports on the RasPi2.

* Connect an Ethernet cable between your network and the RasPi2.

Power up the RasPi2.  It will scroll a whole bunch of stuff
and eventually prompt with `... login:`.  The user name is
`ubuntu` and the password is `ubuntu`.  After you log in,
please do the following:

        sudo apt-get install libnss-mdns
        sudo apt-get install openssh-server

`libnss-mdns` is the "zeroconf" package for networking.
`openssh-server` is a server that will allow you to log
into your RasPi2 over the network.

We want your laptop/desktop to support both "openssh" and
"zeroconf".  So, from your laptop, do the following:

        sudo apt-get install libnss-mdns
        sudo apt-get install openssh-server
        cat /etc/hostname

The last command should print out the hostname of the
laptop/desktop.  In the samples below, we will assume
that the host name is `myhostname`.  Now, we make
sure that we have "zeroconf" working on your desktop/laptop:

        ping myhostname.local

You should get some message that look like:

        PING myhostname.local (129.168.##.##) 56(84) bytes of data.
        64 bytes from 192.168.0.5: icmp_seq=1 ttl=64 time=0.054 ms
        64 bytes from 192.168.0.5: icmp_seq=2 ttl=64 time=0.067 ms
        64 bytes from 192.168.0.5: icmp_seq=3 ttl=64 time=0.062 ms
        ...

If that works, "zeroconf" is working with your laptop/desktop.
Type Control-C to stop the `ping` program.

Now make sure that your laptop/desktop can find the RasPi2.

        ping ubuntu.local

You should get a similar set of messages.  Type Control-C
to stop the `ping` program.

Now we want to log into the RasPi2:

        ssh -l ubuntu unbuntu.local

You should get a dialog that looks roughly as follows:

        The authenticity of host 'ubuntu.local (192.168.##.##)' can't be established.
        ECDSA key fingerprint is d5:f0:41:...:f5.       # A bunch of hex stuff
        Are you sure you want to continue connecting (yes/no)? yes  # Type `yes`
        Warning: Permanently added 'ubuntu.local,192.168.##.' (ECDSA) to the list of known hosts.
        ubuntu@ubuntu.local's password:      # Type `ubuntu` here
        Welcome to Ubuntu 14.04.2 LTS (GNU/Linux 3.18.0-20-rpi2 armv7l)
        ubuntu@ubuntu:~$

If you get this, you have successfully logged in to your RasPi2.
Now we reboot the RasPi2 from your desktop/laptop.  From the
windo that has the `ubuntu@ubuntu:~$` prompt, type:

        sudo reboot
        [sudo] password for ubuntu:     # Type `ubuntu` here

You should get the following:

        Broadcast message from ubuntu@ubuntu
                (/dev/pts/0) at 19:33 ...

        The system is going down for reboot NOW!
        Connection to ubuntu.local closed by remote host.
        Connection to ubuntu.local closed.

Followed by a prompt from your desktop/laptop.

Wait about 60 seconds and the RasPi2 should be rebooted.
Now login to the RasPi2 again:

        ssh -l ubuntu ubuntu.local
        
You should get a shorter message this time:

        ubuntu@ubuntu.local's password:   # Type `ubuntu` here
        Welcome to Ubuntu 14.04.2 LTS (GNU/Linux 3.18.0-20-rpi2 armv7l)

         * Documentation:  https://help.ubuntu.com/
        Last login: Wed Apr 15 20:03:01 2015 from 192.168.1.5
        ubuntu@ubuntu:~$ 

Now you can safely halt the machine with the following command:

        sudo halt
        [sudo] password for ubuntu: 

        Broadcast message from ubuntu@ubuntu
                (/dev/pts/0) at 20:04 ...

        The system is going down for halt NOW!
        ubuntu@ubuntu:~$ Connection to ubuntu.local closed by remote host.
        Connection to ubuntu.local closed.

Now dow the following:

* Remove power from the RasPi2.

* Unplug both your HDMI cable and USB keyboard.

* Leave the Ethernet cable installed.

* Now power up the RasPi2 again.

After 60 seconds or so, log in using:

        ssh -l ubuntu ubuntu.local

Follow the standard instructions for installing ROS from the
  [ROS UbuntuArm](http://wiki.ros.org/indigo/Installation/UbuntuARM)
page:

        sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
        sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros trusty main" > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.namniart.com/repos/namniart.key -O - | sudo apt-key add -
        time sudo apt-get update			# ~1 min
        time sudo apt-get install ros-indigo-ros-base	# ~10 min
        sudo apt-get install python-rosdep
        sudo rosdep init
        time rosdep update				# ~1 min
        echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        time sudo apt-get install python-rosinstall	# ~4 min
	sudo apt-get install build-essential		# ~2 min

To check whether or not ROS is working:

        roscore
	# See whether it prints some stuff out...
        # ... if it does, type Control-C to shut it down.


Install the arduino stuff:

        sudo apt-get install avrdude-doc gcc-avr avrdude avr-libc binutils-avr
        sudo apt-get install libcv-dev libopencv-dev

Right now we configure WiFi by editing the `/etc/network/interfaces` file:

        sudo vi /etc/network/interfaces

Add the following:

        allow-hotplug wlan0
        iface wlan0 inet dhcp
        wpa-essid "YourNetworkID"
        wpa-psk "yourSecretPassword"

Now plug in the USB WiFi dongle.  Only the RealTek 8192CU dongle
has been tested.

        ifconfig
        # Should list devices `lo`, `eth0`, and `wlan0`

Now shut down `eth0` and `wlan0`:

        sudo ifdown eth0
        sudo ifdown wlan0

Now bring up only `wlan0`:

        sudo ifup wlan0

Verify that everything workds:

        ping google.com
        # Type control-C after a few lines come through

To log onto the matchine over the WiFi:

        ssh -l linaro raspberry.local
        {password is 'linaro'}

Done.

## ROS on Raspberry Pi 2 (OLD)

*** This section is no longer useful.  Read the previous section. ***

We are going to shorten Raspberry Pi 2 down to RasPi2
just to save a little typing.

Currently (as of Feb2015), the only two precompiled systems
for the RasPi2 are
[Raspian and Ubuntu Snappy](http://www.raspberrypi.org/downloads/).
Ubuntu Snappy has a new package manager called snappy which
is not based on Debian packages.  Currently, ROS only runs
on Ubuntu with Debian packages.  Thus, Ubuntu Snappy is not
compatible with ROS.  While
[OSRF intends to support Ubuntu Snappy](http://www.osrfoundation.org/ubuntu-ros-apps-on-the-way/), it may take a while to work out all of the issues.

So, for now (Feb2015), that leaves us with the Raspian release.
Raspian boots just fine on the RasPi2.  The issue is that
the Debian releases (Wheezy and Jesse) put files in different
locations than the Ubuntu releases.  Thus, while both Ubuntu
and Raspian use Debian packages, the Packages are not compatible
with one another.  Bummer!

So, the basic strategy is to construct a hybrid system
that uses the Raspian Linux kernel in conjunction with
the Ubunutu 14.04 LTS release.  The Raspian release is
running Linux 3.18, where as the Ubuntu 14.04 release
is running 3.14.  The only reason why we have a chance
of making this work is because the Linux kernel team works
incredibly hard to keep Linux compatible from release
to release.  (If a kernel developer breaks Linux User
space, Linus Torvolds lets his displeasure be known.)

### Required Hardware

In order to pull this all off we need the following:

* A RasPi2 with appropriate power supply and network cable.
* A desktop (or laptop) machine with an internet connection.
* Two micro-SD cards that are at least 8GB in size.
* A USB to micro-SD card adaptor.
* An HDMI monitor
* A USB Keyboard.

### Overview

So here is an overview of what is going to happen:

* We will download the Raspian release onto a desktop
  machine over network.

* The Raspian release will be copied onto both Micro-SD
  cards.  One card will be labeled "Raspian" an the other
  will be labeled "Hybrid".

* Each Micro-SD card will be booted, updated and upgraded
  and shut down using the network connection and the USB
  serial cable to access the console window.  (This may
  change.)

* Download minimal Ubuntu system onto the Raspian micro-SD card
  running on the RasPi2 in to a directory called `ubuntu` using
  the `debootstrap` program.

* Mount the Hybrid micro-SD card onto the Raspberry Pi 2 system
  using the USB to micro-SD card.

* Delete everything but the kernel files from Hybrid Micro-SD
  card.

* Copy everything but the kernel from the `ubuntu` directory
  over to the hybrid Micro-SD, shut down Raspaian and boot
  Hybrid.  We keep swapping between Hybrid and Raspian until
  we are happy with everything.

The article
[6 Steps for Minimal Ubuntu Installation Using debootstrap](http://www.thegeekstuff.com/2010/01/debootstrap-minimal-debian-ubuntu-installation/)
provides some additional insight.

### Install Raspian to Micro-SD Cards

Perform the following steps:

* Obtain two micro-SD cards.  Label one "Raspian" and the other
  "Hybrid".

* Download the Raspian Debian Wheezy release from
  [Raspberry Pi 2 Downloads web page](http://www.raspberrypi.org/downloads/).

* Unzip the downloaded `.zip` file to get `2015-01-32-raspian.img`.

* Using the `dd` command copy the image to the "Raspian"
  micro-SD card.  (See the section above on the Beaglebone Black
  about how to use the `dd` command to copy the image to the
  micro-SD card.)  This can take 10-20 minutes depending up various factors.

* Do the same `dd` command to copy the same image to "Hybrid"
  micro-SD card.

* Plug the HDMI monitor and USB keyboard into the RasPi2.
  Insert the "Raspian" micro-SD card into the RasPi2.
  Power up the RasPi2.

* You should see the boot messages on the screen and it should
  eventually prompt you with `login:`.  You can log in as user `pi`.
  with a password of `raspberry`.

* Expand the disk.  (This could be more descriptive.)

* Connect one end of the network cable to the RasPi2 and the
  other to your network.

* Perform the following commands to update the repository
  list and upgrade the packages:

        sudo apt-get update
        sudo apt-get upgrade

* Leave the RasPi2 running for the next set of steps:

### Download Ubuntu to the "Raspian" micro-SD:

* Create a `ubuntu` directory:

        cd ~
        mkdir ubuntu
        cd ubuntu

* Download `debootstrap` and install it:

        sudo apt-get install debootstrap
        sudo apt-get install build-essential

* Run the debootstrap command:

        # Get the debootstrap command arguments from Mike

* Copy `/etc/fstab`:

        cp /etc/fstab ~/ubuntu/etc

* Copy and edit `/etc/resolv.conf`

        cp /etc/resolv.conf ~/ubuntu/resolve.conf

  Notice that `resolv` is spelled without an `e`.  (It is an
  old archaic Unix thing.)  Edit `/etc/resolv` so that the two
  `nameserver` lines look like:

        nameserver 8.8.8.8
        nameserver 8.8.4.4

* Copy and edit `/etc/hostname`:

        cp /etc/hostname ~/ubuntu/hostname

  If you want to change the hostname from `pi` to something
  else please do so.

* Copy and edit `/etc/hosts`:

        cp /etc/hosts ~/ubuntu/hosts

  Edit `/etc/hosts` to change all occurrences of `pi` to
  your new host name.  (See previous step.)

* Mount `/proc` on `~/ubuntu/proc`:

        mkdir -p ~/ubuntu/proc
        sudo mount -t proc ~/ubuntu/proc /proc

  This command makes it possible to access the network
  while running `chroot`.

* Run the `chroot` command:

        # Is this right?
        chroot ~/ubuntu

* Now it is possible to run the `apt-get` command and
  install ubuntu packages into the ~/ubuntu/ file system.
  (This is really, really sneaky!!!!)

  Install the following packages:

        sudo apt-get install sudo net-tools vim
        sudo apt-get install openssh-client openssh-server ping
        # What else needs to be installed?

* Manually install the following debian packages:

        sudo mkdir /kernel
        cd /kernel
        export DEBHOST=http://archive.raspberrypi.org/debian/pool/main/r/raspberrypi-firmware
        export DEBSUFFIX=1.20150214-1_armhf.deb
        sudo wget $DEBHOST/libraspberrypi-bin_$DEBSUFFIX
        sudo wget $DEBHOST/libraspberrypi-dev_$DEBSUFFIX
        sudo wget $DEBHOST/libraspberrypi-doc_$DEBSUFFIX
        sudo wget $DEBHOST/libraspberrypi0_$DEBSUFFIX
        sudo wget $DEBHOST/raspberrypi-bootloader_$DEBSUFFIX
        sudo dpkg -i *.deb

  There may be some issues to resolve with the commands above.

* Create new user:

  Note, substitute the new user account name for `USER` in
  the command below:

        # Use the `adduser` command here:
        sudo adduser --home /home/USER
        # Provide passwords here.

* Exit `chroot`:

        exit

* Unmount ~/ubuntu/proc:

        umount ~/ubuntu/proc


### Transfer the Files Over to the Hybrid micro-SD.

Now we transfer the Ubuntu files from the Raspian micro-SD
to the Hybrid micro-SD:

* Put the Hybrid micro-SD into the USB to micro-SD adapter.

* Mount the Hybrid micro-SD onto RasPi2 (which is
 still running off the Raspian micro-SD.)

        sudo mkdir -p /mnt/hybrid
        sudo mount /dev/?? /mnt/hybrid 
        cd /mnt/hybrid 

* Delete everything but the `/lib/modules ` and `/boot` directories.

        # Where is the modules directory?
        sudo -s
        cd /mnt/hybrid

        # Hang onto `/lib/modules`:
        mkdir -p boot/lib_modules
        mv lib/modules/ boot/lib_modules

        # Make darn sure you are deleting from the /mnt/hybrid:
        cd /mnt/hybrid
        rm -rf bin dev etc home lib media opt proc sbin sys usr var

        # Restore some directories:
        mkdir -p mnt proc lib

        # Restore `/lib/modules`
        mv boot/lib_modules lib/modules

* Copy the contents of `~/ubuntu` over to `~/mnt/hybrid`

        sudo tar cf - ~/ubuntu  | (cd ~/mnt/hybrid; tar xf -)

* Missing steps:

  * Add `pi` user to `/etc/sudoers`
  * What else?

* Unmount the Hybrid micro-SD:

        sudo umount /mnt/hybrid

* Shut down the RasPi2 running the Raspian micro-SD.

        sudo halt


### Boot the hybrid micro-SD:

* Install the Hybrid micro-SD into RasPi2.

* Boot it.

* Read the boot log.

* Log in as `pi` with password of `pi`.

### Networking:

To manually mount the `eth0` network adapter from the
command line:

        ifconfig eth0 192.168.x.y netmask 255.255.255.0
        route add default gw 192.168.x.z eth0

To configure networking to boot up automatically edit
`/etc/network/interfaces` to have the following lines:

        auto eth0
        iface eth0 inet static
        address 192.168.x.y
        gateway 192.168.x.z

Comment out or delete all other lines concerning `eth0`.
When you reboot, the wired ethernet connection should
come up automatically.

### Installing ROS

Update `/etc/apt/sources.list` with:

        # See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
        # newer versions of the distribution.
        
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
        
        ## Major bug fix updates produced after the final release of the
        ## distribution.
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
        
        ## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
        ## team. Also, please note that software in universe WILL NOT receive any
        ## review or updates from the Ubuntu security team.
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty universe
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe
        
        ## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu 
        ## team, and may not be under a free licence. Please satisfy yourself as to 
        ## your rights to use the software. Also, please note that software in 
        ## multiverse WILL NOT receive any review or updates from the Ubuntu
        ## security team.
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty multiverse
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty multiverse
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates multiverse
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-updates multiverse
        
        ## N.B. software from this repository may not have been tested as
        ## extensively as that contained in the main release, although it includes
        ## newer versions of some applications which may provide useful features.
        ## Also, please note that software in backports WILL NOT receive any review
        ## or updates from the Ubuntu security team.
        # deb http://ports.ubuntu.com/ubuntu-ports/ trusty-backports main restricted universe multiverse
        # deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-backports main restricted universe multiverse
        
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security main restricted
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security main restricted
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security universe
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security universe
        deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security multiverse
        deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security multiverse

Follow Austin's directions.

