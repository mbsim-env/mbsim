#!/bin/sh

# Build feeds
/home/user/Tools/rsstool-1.0.1rc2-src/src/rsstool --rss -o=/media/mbsim-env/mergedFeeds/build.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux/report/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux/report_distribute/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows/report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows/report/runexamples_report/result.rss.xml \
http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows/report_distribute/result.rss.xml

# google code feeds
/home/user/Tools/rsstool-1.0.1rc2-src/src/rsstool --rss -o=/media/mbsim-env/mergedFeeds/googlecode.rss.xml \
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
