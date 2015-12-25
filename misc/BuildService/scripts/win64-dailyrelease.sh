#! /bin/sh

SRCDIR=/home/mbsim/win64-dailyrelease
OUTDIR=/var/www/html/mbsim/win64-dailyrelease
URL=http://www.mbsim-env.de/mbsim/win64-dailyrelease
SCRIPTDIR=$(dirname $(realpath $0))


# mfmf replace rss/atom/feed with addBuildSystemFeed
rss() {
DATE1=$(date +%s)
DATE2=$(date -R)
cat << EOF > $OUTDIR/report_distribute/result.rss.xml
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>Win64 Daily Release</title>
    <link>$URL/report_distribute/distribute.out</link>
    <description>Win64 Daily Release: Result RSS feed of the MBSim daily Win64 build/distribution</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="$URL/report_distribute/result.rss.xml" rel="self" type="application/rss+xml"/>
EOF
if [ $1 -eq 1 ]; then
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Win64 Daily Release: creating distribution failed</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_$DATE1</guid>
      <pubDate>$DATE2</pubDate>
    </item>
EOF
fi
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Win64 Daily Release: Dummy feed item. Just ignore it.</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>
EOF
}

rss 0

export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-win64/lib/pkgconfig:/home/mbsim/3rdparty/coin-local-win64/lib/pkgconfig:/usr/i686-w64-mingw32/sys-root/mingw/lib/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/mbsim/3rdparty/casadi-local-win64/lib
export CXXFLAGS="-g -O2"
export CFLAGS="-g -O2"
export FFLAGS="-g -O2"
export PLATFORM=Windows # required for source code examples
export PATH=$PATH:/home/mbsim/3rdparty/casadi-local-win32/lib
export CXX=i686-w64-mingw32-g++ # required for source code examples

$(dirname $0)/build.py --rotate 14 -j 2 --sourceDir $SRCDIR --prefix $SRCDIR/local --reportOutDir $OUTDIR/report --url $URL/report --buildType "Win64 Daily Release: " --passToConfigure --enable-shared --disable-static --build=i686-pc-linux-gnu --host=i686-w64-mingw32 --with-blas-lib-prefix=$SRCDIR/local/lib --with-qwt-inc-prefix=/usr/i686-w64-mingw32/sys-root/mingw/include/qwt --with-qwt-lib-name=qwt5 --with-mkoctfile=$SRCDIR/local/bin/mkoctfile.exe --with-boost-thread-lib=boost_thread-gcc47-mt-1_48 --with-boost-timer-lib=boost_timer-gcc47-1_48 --with-boost-chrono-lib=boost_chrono-gcc47-1_48 --with-boost-filesystem-lib=boost_filesystem-gcc47-1_48 --with-boost-system-lib=boost_system-gcc47-1_48 --with-boost-locale-lib=boost_locale-gcc47-1_48 --with-boost-date-time-lib=boost_date_time-gcc47-1_48 --with-swigpath=/home/mbsim/Updates/local/bin --with-windres=i686-w64-mingw32-windres --with-javajnicflags=-I/home/mbsim/win64-dailyrelease/jni_include CPPFLAGS=-DSOQT_DLL PKG_CONFIG_PATH=/usr/i686-w64-mingw32/sys-root/mingw/lib/pkgconfig:$SRCDIR/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-win32/lib/pkgconfig PYTHON_CFLAGS=-I/home/mbsim/win64-dailyrelease/3rdparty/python/include PYTHON_LIBS="-L/home/mbsim/win64-dailyrelease/3rdparty/python/libs -lpython27" PYTHON_BIN=/home/mbsim/win64-dailyrelease/3rdparty/python/python.exe --passToRunexamples --disableCompare --disableValidate --exeExt .exe xmlflat/hierachical_modelling xml/hierachical_modelling xml/time_dependent_kinematics xml/hydraulics_ballcheckvalve fmi/simple_test fmi/hierachical_modelling fmi/sphere_on_plane mechanics/basics/hierachical_modelling mechanics/basics/time_dependent_kinematics

RET=0

$(dirname $0)/win64-dailyrelease-distribute.sh &> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1

cp $SRCDIR/dist_mbsim/mbsim-env-win64-shared-build-xxx.zip $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
cp $SRCDIR/dist_mbsim/mbsim-env-win64-shared-build-xxx-debug.zip $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1

if [ $RET -ne 0 ]; then
  rss 1
fi


$SCRIPTDIR/mergeFeeds.py
