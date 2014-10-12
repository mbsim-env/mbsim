#! /bin/sh

# update build scripts
svn update $(dirname $0)/build-scripts

# update references of examples
CURDIR=$(pwd)
SRCDIR=$(dirname $0)
export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig
cd $SRCDIR/mbsim/examples
./runexamples.py --action copyToReference @/home/user/Tools/runexamples-refupdate-cgi.py.conf/updateList # update reference
echo -n "" > /home/user/Tools/runexamples-refupdate-cgi.py.conf/updateList # remove all entries from updateList
cd $CURDIR

# build and run all examples
export CXXFLAGS="-O1 -g"
export CFLAGS="-O1 -g"
$(dirname $0)/build-scripts/daily-build/build.py "$@" --rotate 14 -j 2 --forceBuild --sourceDir /home/user/MBSimDailyBuild --prefix /home/user/MBSimDailyBuild/local --docOutDir /var/www/html/mbsim-env/MBSimDailyBuild/doc --reportOutDir /var/www/html/mbsim-env/MBSimDailyBuild/report --url http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report --buildType "Daily Build: " --passToConfigure --enable-debug --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt --with-boost-locale-lib=boost_locale-mt

# run examples with valgrind
cd $(dirname $0)
SRCDIR=$(pwd)
export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig
cd $SRCDIR/mbsim/examples_valgrind
svn update
MBSIM_SET_MINIMAL_TEND=1 ./runexamples.py --rotate 14 -j 2 --reportOutDir /var/www/html/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --url http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --prefixSimulationKeyword=VALGRIND --prefixSimulation "valgrind --error-exitcode=200 --trace-children=yes --num-callers=150 --gen-suppressions=all --suppressions=$SRCDIR/build-scripts/daily-build/valgrind-mbsim.supp --leak-check=full --show-reachable=yes" --disableCompare --disableValidate --buildType "Daily Build valgrind: "
