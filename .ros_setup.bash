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
