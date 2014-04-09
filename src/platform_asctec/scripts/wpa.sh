#!/bin/bash
### BEGIN INIT INFO
# Provides:          wpa
# Required-Start:    $network $syslog $local_fs
# Required-Stop:     $network $syslog $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start/stop script for wpa supplicant
# Description:       Custom start/stop script for wpa_supplicant.
### END INIT INFO

SELF=`basename $0`
WPA=wpa_supplicant
PROGRAM=/sbin/${WPA}
CONF=/etc/${WPA}.conf
INTERFACE=wlan0
DRIVER=wext
DAEMONMODE="-B"
LOGFILE=/var/log/$WPA.log

function start() {

    # TODO: Support multiple interfaces and drivers
    OPTIONS="-c $CONF -i $INTERFACE -D $DRIVER $DAEMONMODE"

    ## You can remove this if you are running 8.10 and up.
    # Ubuntu 8.10 and up doesn't need the -w anymore..
    # And the logfile option is not valid on 8.04 and lower
    local ver=$(lsb_release -sr | sed -e 's/\.//g');
    [ $ver -lt 810 ] && OPTIONS="$OPTIONS -w" && LOGFILE=""
    ##

    # Log to a file
    [ -n "$LOGFILE" ] && OPTIONS="$OPTIONS -f $LOGFILE"

    echo " * Starting wpa supplicant"
    eval $PROGRAM $OPTIONS
}

function stop() {
    echo " * Stopping wpa supplicant"
    pkill $PROGRAM
}

function debug() {
    stop
    DAEMONMODE="-ddd"
    start
}

function restart() {
    stop
    start
}

function status() {
    pgrep -lf $PROGRAM
}

function usage() {
    echo "Usage: $SELF <start|stop|status|debug>"
    return 2
}

case $1 in
    start|stop|debug|status) $1 ;;
    *) usage ;;
esac