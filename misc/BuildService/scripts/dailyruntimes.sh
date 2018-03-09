#!/bin/sh

for B in linux64-dailydebug linux64-dailyrelease win64-dailyrelease; do
  D=/var/www/html/mbsim/$B/report/result_current
  ID=$(grep ">Time ID<" $D/index.html | sed -re "s/.*dd>([^<]+)<.*/\1/" | sed -re "s/ .*//")
  S=$(date +%s -d "$(grep ">Time ID<"  $D/index.html | sed -re "s/.*dd>([^<]+)<.*/\1/")")
  E=$(date +%s -d "$(grep ">End time<" $D/index.html | sed -re "s/.*dd>([^<]+)<.*/\1/")" 2> /dev/null)
  echo "$ID = $[($E-$S)/60] min" >> $HOME/log/buildtimes/$B
done
