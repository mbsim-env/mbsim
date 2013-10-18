#! /bin/sh

# update build scripts
svn update $(dirname $0)/build-scripts

# build and run all examples
$(dirname $0)/build-scripts/daily-build/build.py "$@" -j 2 --forceBuild --sourceDir /home/user/MBSimDailyBuild --prefix /home/user/MBSimDailyBuild/local --docOutDir /media/mbsim-env/MBSimDailyBuild/doc --reportOutDir /media/mbsim-env/MBSimDailyBuild/report --url http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report --buildType "Daily Build: " --passToConfigure --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt

# run some examples with valgrind
cd $(dirname $0)
SCRDIR=$(pwd)
export PKG_CONFIG_PATH=$SCRDIR/local/lib/pkgconfig
cd $SCRDIR/mbsim/examples
MBSIM_SET_MINIMAL_TEND=1 ./runexamples.py -j 2 --reportOutDir /media/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --url http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --prefixSimulation "valgrind --error-exitcode=200 --trace-children=yes --num-callers=50" --disableCompare --disableValidate --buildType "Daily Build valgrind: "
