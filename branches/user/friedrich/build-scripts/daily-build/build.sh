#! /bin/sh

# update build scripts
svn update $(dirname $0)/build-scripts

# build and run all examples
$(dirname $0)/build-scripts/daily-build/build.py "$@" --rotate 30 -j 2 --forceBuild --sourceDir /home/user/MBSimDailyBuild --prefix /home/user/MBSimDailyBuild/local --docOutDir /media/mbsim-env/MBSimDailyBuild/doc --reportOutDir /media/mbsim-env/MBSimDailyBuild/report --url http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report --buildType "Daily Build: " --passToConfigure --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt --with-boost-locale-lib=boost_locale-mt

# run examples with valgrind
cd $(dirname $0)
SRCDIR=$(pwd)
export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig
cd $SRCDIR/mbsim/examples_valgrind
svn update
MBSIM_SET_MINIMAL_TEND=1 ./runexamples.py --rotate 30 -j 2 --reportOutDir /media/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --url http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --prefixSimulation "valgrind --error-exitcode=200 --trace-children=yes --num-callers=50 --gen-suppressions=all --suppressions=$SRCDIR/build-scripts/daily-build/valgrind-mbsim.supp --leak-check=full" --disableCompare --disableValidate --buildType "Daily Build valgrind: "
