#!/bin/sh

WEBDIR=/var/www/html/mbsim-env/mergedFeeds

# Build feeds
/home/user/bin/rsstool-1.0.1rc2-src/src/rsstool -r --rss -o=$WEBDIR/build2.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimContinuousIntegration/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimContinuousIntegration/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimLinux/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimLinux/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimLinux/report_distribute/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimWindows/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimWindows/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimWindows/report_distribute/result.rss.xml
# fix CEST to CET (CEST does not work on all clients)
sed -re "s|<pubDate>(.*)CEST</pubDate>|<pubDate>\1CET</pubDate>|g" < $WEBDIR/build2.rss.xml > $WEBDIR/build.rss.xml
