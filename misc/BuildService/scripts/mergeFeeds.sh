#!/bin/sh

WEBDIR=/var/www/html/mbsim/mergedFeeds

# Build feeds
/home/mbsim/3rdparty/rsstool-1.0.1rc2-src/src/rsstool -r --rss -o=$WEBDIR/build2.rss.xml \
/var/www/html/mbsim/linux64-dailydebug/report/result.rss.xml \
/var/www/html/mbsim/linux64-dailydebug/report/result_current/runexamples_report/result.rss.xml \
/var/www/html/mbsim/linux64-dailydebug/report/runexamples_valgrind_report/result.rss.xml \
/var/www/html/mbsim/linux64-ci/report/result.rss.xml \
/var/www/html/mbsim/linux64-ci/report/result_current/runexamples_report/result.rss.xml \
/var/www/html/mbsim/linux64-dailyrelease/report/result.rss.xml \
/var/www/html/mbsim/linux64-dailyrelease/report/result_current/runexamples_report/result.rss.xml \
/var/www/html/mbsim/linux64-dailyrelease/report_distribute/result.rss.xml \
/var/www/html/mbsim/win64-dailyrelease/report/result.rss.xml \
/var/www/html/mbsim/win64-dailyrelease/report/result_current/runexamples_report/result.rss.xml \
/var/www/html/mbsim/win64-dailyrelease/report_distribute/result.rss.xml
# fix CEST to CET (CEST does not work on all clients)
sed -re "s|<pubDate>(.*)CEST</pubDate>|<pubDate>\1CET</pubDate>|g" < $WEBDIR/build2.rss.xml > $WEBDIR/build.rss.xml
