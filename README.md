# ROS Notes

These are some notes taken during the process of figuring out
how to use ROS (Robot Operating System)

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
         *{ your_name: your question ... *}
         <BlockQuote>

## ROS Concepts

A good place to start is
  [ROS/Concepts](http://wiki.ros.org/ROS/Concepts)
wiki page.  The meat of the ROS concepts is in the section
entitled "ROS Computational Graph Level".

ROS is really a meta operating system that is layered on
top of a more traditional operating system.  As of early
2015, the primary operating system is Linux, although there
has been some effort to port ROS to run on top of MacOS
and Windows.

ROS also heavily uses Debian packages.  Thus, it most easily
runs on top of Linux distributions that use Debian packages,
particularly the Ubuntu distribution.

ROS decomposes software into a confederation of computational
processes called *nodes*.  The confederation of nodes use
strongly typed message passing to communicate with one another.
There are two styles of message passing:

* *ROS Topic*: A ROS topic is basically a broadcast mechanism
  where one one or more ROS nodes can broadcast strongly
  typed messages.  There can be more than one broadcaster
  and there can be more than one listener.

* *ROS Service*: A ROS service is a more traditional remote
  procedure call.  It looks like there can be multiple callers,
  but only one callee.

There are some additional observations about the ROS
computational model:

* *Language Agnostic*: Each node can be written in any
  programming language that supports the message passing.
  At this time, the two primary supported languages are
  C++ and Python.  Other languages can be added by writing
  a library that supports the ROS message passing protocols.

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
  robot uses one of these Linux real time extensions for
  some of their nodes.)

* *Non-deterministic*: The underlying operating system
  can schedule nodes for execution in an arbitrary order.
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

## Installing ROS

ROS has various releases.  At this time, the two releases
are *Hydro* and *Indigo*.  It is important that all machines
involved with a robot run the same version.  Each ROS release
is targeted to one or more Ubuntu releases.  For example,
Hydro runs on Ubuntu 12.04 or 13.04.  Whereas, Indigo runs
on 14.04.

Ubuntu uses a numbering scheme of YY.MM, where YY is the
last two digits of the year, and MM is the month number.
The two month dates are either 04 (April) or 10 (October).
The April releases (i.e. 04) on even years are LTS 
(Long Term Support) releases that supply bug patches
for 5 years, whereas all other releases are called a
"standard release" and only provide bug patches for about
six months.  Thus, 12.04 and 14.04 are LTS releases.

ROS supports having multiple releases installed on the
same machine.  The way to switch between releases is to
run the `/opt/ros/`*release_name*`/setup.bash` script.
This actually is pretty cool.

The robot will almost certainly be running a Ubuntu
Linux distribution using either the *x86* or *ARM*
hardware architecture.  

The debugging/control/visualization machine can be run some
non-Linux native operating system (e.g. Windows, MacOS,
Solaris), but it will almost certainly need to be running
some sort of virtualization software such as VMWare,
VirtualBox, Parallels, etc.  VirtualBox is free and
runs on both Windows and MacOS.

There appear to be plenty of resources available for
installing ROS.  The
  [ROS/Installation](http://wiki.ros.org/ROS/Installation)
wiki page is a good place to start.

Section 4.12 of
  [Ros By Example Hydro - Volume 1](http://www.lulu.com/shop/search.ep?contributorId=1109044)
by Patrick Goebel discusses how to set up ROS to work over
the internet.  Alas, missing from this discussion, is an
explanation of how to configure the Ubuntu running on the
robot to
  [automatically connect](http://thenewbieblog.wordpress.com/2012/05/01/wifi-hotspot-setup-on-ubuntu/)
to a wireless access point.

## ROS Introductory Tutorials

Pretty much every starts with the standard
  [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) .

Probably what comes next is to go the through the tf
[tf (transforms)](http://wiki.ros.org/tf/Tutorials).

