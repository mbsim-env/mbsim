#!/bin/sh

function killStoppedProcesses() {
  for CMD in $1; do
    for PID in $(pidof $CMD); do
      if [ $[($(date +%s)-$(date -d "$(ps -p $PID -o lstart=)" +%s))/60/60] -gt $2 ]; then
        echo kill $PID
        sleep 10s
        echo kill -9 $PID
      fi
    done
  done
}

# Kill all build.sh processes after 5 hours uptime
COMMANDS="build.sh"
MAXTIME=5 # in hours
killStoppedProcesses "$COMMANDS" $MAXTIME

# Kill all release.sh processes after 4 hours uptime
COMMANDS="release.sh"
MAXTIME=4 # in hours
killStoppedProcesses "$COMMANDS" $MAXTIME

# Kill all mergeFeeds.sh and googleCode-mirror.sh processes after 1 hours uptime
COMMANDS="mergeFeeds.sh googleCode-mirror.sh"
MAXTIME=1 # in hours
killStoppedProcesses "$COMMANDS" $MAXTIME
