#! /bin/sh

# update build scripts
svn update $(dirname $0)/build-scripts

# build and run all examples
$(dirname $0)/build-scripts/daily-build/build.py "$@" --forceBuild --sourceDir /home/user/MBSimDailyBuild --prefix /home/user/MBSimDailyBuild/local --docOutDir /media/mbsim-env/MBSimDailyBuild/doc --reportOutDir /media/mbsim-env/MBSimDailyBuild/report --url http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimDailyBuild/report --buildType "Daily Build: " --passToConfigure --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt --with-javacflags="-I/usr/lib/jvm/java-1.7.0-openjdk-1.7.0.19/include -I/usr/lib/jvm/java-1.7.0-openjdk-1.7.0.19/include/linux"

# run some examples with valgrind
cd $(dirname $0)
SCRDIR=$(pwd)
export PKG_CONFIG_PATH=$SCRDIR/local/lib/pkgconfig
cd $SCRDIR/mbsim/examples
./runexamples.py --reportOutDir /media/mbsim-env/MBSimDailyBuild/report/runexamples_valgrind_report --prefixSimulation "valgrind --error-exitcode=-30000 --trace-children=yes --num-callers=50" mechanics/basics/hierachical_modelling xmlflat/hierachical_modelling xml/hierachical_modelling mechanics/basics/time_dependent_kinematics xmlflat/time_dependent_kinematics xml/time_dependent_kinematics
