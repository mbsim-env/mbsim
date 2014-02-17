#!/bin/sh

WEBDIR=/media/mbsim-env/mergedFeeds

# Build feeds
/home/user/Tools/rsstool-1.0.1rc2-src/src/rsstool -r --rss -o=$WEBDIR/build2.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux/report_distribute/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows/report/result_current/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows/report_distribute/result.rss.xml
# fix CEST to CET (CEST does not work on all clients)
sed -re "s|<pubDate>(.*)CEST</pubDate>|<pubDate>\1CET</pubDate>|g" < $WEBDIR/build2.rss.xml > $WEBDIR/build.rss.xml

# google code feeds
/home/user/Tools/rsstool-1.0.1rc2-src/src/rsstool -r --rss -o=$WEBDIR/googlecode2.rss.xml \
http://code.google.com/feeds/p/fmatvec/downloads/basic \
http://code.google.com/feeds/p/fmatvec/issueupdates/basic \
http://code.google.com/feeds/p/fmatvec/svnchanges/basic \
http://code.google.com/feeds/p/hdf5serie/downloads/basic \
http://code.google.com/feeds/p/hdf5serie/issueupdates/basic \
http://code.google.com/feeds/p/hdf5serie/svnchanges/basic \
http://code.google.com/feeds/p/openmbv/downloads/basic \
http://code.google.com/feeds/p/openmbv/issueupdates/basic \
http://code.google.com/feeds/p/openmbv/svnchanges/basic \
http://code.google.com/feeds/p/mbsim-env/downloads/basic \
http://code.google.com/feeds/p/mbsim-env/issueupdates/basic \
http://code.google.com/feeds/p/mbsim-env/svnchanges/basic
# fix CEST to CET (CEST does not work on all clients)
sed -re "s|<pubDate>(.*)CEST</pubDate>|<pubDate>\1CET</pubDate>|g" < $WEBDIR/googlecode2.rss.xml > $WEBDIR/googlecode.rss.xml
