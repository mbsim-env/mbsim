#! /bin/sh

svn update $(dirname $0)/build-scripts

$(dirname $0)/build-scripts/daily-build/build.py "$@" --forceBuild --sourceDir /home/user/MBSimDailyBuild --prefix /home/user/MBSimDailyBuild/local --docOutDir /media/mbsim-env/MBSimDailyBuild/doc --reportOutDir /media/mbsim-env/MBSimDailyBuild/report --url http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report --buildType "Daily Build: " --passToConfigure --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt
