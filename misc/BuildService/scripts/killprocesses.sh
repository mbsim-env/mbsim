#!/bin/sh

function killStoppedProcesses() {
  for CMD in $1; do
    for PID in $(/usr/sbin/pidof -x $CMD); do
      if [ $[($(date +%s)-$(date -d "$(ps -p $PID -o lstart=)" +%s))/60/60] -gt $2 ]; then
        ALLPIDS=$(pstree -lAp $PID | sed -re "s/-[\+-]-/\n/g" | sed -re "s/.*\(([0-9]+)\).*/\1/g")
        echo "$(date) KILL $ALLPIDS"
        kill $ALLPIDS
        sleep 10s
        kill -9 $ALLPIDS
      fi
    done
  done
}

COMMANDS="build-linux-debug.sh build-linux-release.sh build-windows-release.sh"
MAXTIME=5 # in hours
killStoppedProcesses "$COMMANDS" $MAXTIME

COMMANDS="build-linux-ci.py"
MAXTIME=2 # in hours
killStoppedProcesses "$COMMANDS" $MAXTIME

COMMANDS="mergeFeeds.sh builddoc.sh"
MAXTIME=1 # in hours
killStoppedProcesses "$COMMANDS" $MAXTIME
