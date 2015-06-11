#! /bin/sh

# update references of examples
CURDIR=$(pwd)
SRCDIR=$(dirname $0)/../../../..
export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig:/home/user/3rdparty/casadi-local-linux32/lib/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/user/3rdparty/casadi-local-linux32/lib
cd $SRCDIR/mbsim/examples
./runexamples.py --action copyToReference @/home/user/BuildServiceConfig/mbsimBuildService.conf # update reference
cd $CURDIR

# build and run all examples
export CXXFLAGS="-O0 -g"
export CFLAGS="-O0 -g"
$(dirname $0)/build.py "$@" --rotate 14 -j 2 --sourceDir /home/user/MBSimDailyBuild --prefix /home/user/MBSimDailyBuild/local --docOutDir /var/www/html/mbsim-env/MBSimDailyBuild/doc --reportOutDir /var/www/html/mbsim-env/MBSimDailyBuild/report --url http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report --buildType "Daily Build: " --passToConfigure --enable-debug --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt --with-boost-locale-lib=boost_locale-mt --with-swigpath=/home/user/Updates/local/bin

# update references for download
cd $SRCDIR/mbsim/examples
./runexamples.py --action pushReference=/var/www/html/mbsim-env/MBSimDailyBuild/references
cd $CURDIR

# run examples with valgrind
cd $(dirname $0)/../../../..
SRCDIR=$(pwd)
export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig:/home/user/3rdparty/casadi-local-linux32/lib/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/user/3rdparty/casadi-local-linux32/lib
cd $SRCDIR/mbsim_valgrind/examples
git pull
MBSIM_SET_MINIMAL_TEND=1 ./runexamples.py --rotate 14 -j 2 --reportOutDir /var/www/html/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --url http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --prefixSimulationKeyword=VALGRIND --prefixSimulation "valgrind --trace-children=yes --trace-children-skip=*/rm,*/patchelf --num-callers=150 --gen-suppressions=all --suppressions=$SRCDIR/mbsim_valgrind/misc/BuildService/scripts/valgrind-mbsim.supp --leak-check=full" --disableCompare --disableValidate --buildType "Daily Build valgrind: "
