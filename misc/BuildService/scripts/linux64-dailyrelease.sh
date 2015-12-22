#! /bin/sh

SRCDIR=/home/mbsim/linux64-dailyrelease
OUTDIR=/var/www/html/mbsim/linux64-dailyrelease
URL=http://www.mbsim-env.de/mbsim/linux64-dailyrelease
SCRIPTDIR=$(dirname $(realpath $0))



rss() {
DATE1=$(date +%s)
DATE2=$(date -R)
cat << EOF > $OUTDIR/report_distribute/result.rss.xml
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>Linux64 Daily Release</title>
    <link>$URL/report_distribute/distribute.out</link>
    <description>Linux64 Daily Release: Result RSS feed of the MBSim daily Linux64 build/distribution</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="$URL/report_distribute/result.rss.xml" rel="self" type="application/rss+xml"/>
EOF
if [ $1 -eq 1 ]; then
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Linux64 Daily Release: creating distribution failed</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_$DATE1</guid>
      <pubDate>$DATE2</pubDate>
    </item>
EOF
fi
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Linux64 Daily Release: Dummy feed item. Just ignore it.</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>
EOF
}

rss 0

rm -rf $SRCDIR/local/share/mbxmlutils

export PKG_CONFIG_PATH=$SRCDIR/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-linux64/lib/pkgconfig:/home/mbsim/3rdparty/coin-local-linux64/lib/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/mbsim/3rdparty/casadi-local-linux64/lib
export CXXFLAGS="-g -O2"
export CFLAGS="-g -O2"
export FFLAGS="-g -O2"

$(dirname $0)/build.py --rotate 14 -j 2 --sourceDir $SRCDIR --prefix $SRCDIR/local --reportOutDir $OUTDIR/report --url $URL/report --buildType "Linux64 Daily Release: " --passToConfigure --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt --with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin --with-javajnicflags="-I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.37.x86_64/include -I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.37.x86_64/include/linux" --passToRunexamples --disableCompare --disableValidate xmlflat/hierachical_modelling xml/hierachical_modelling xml/time_dependent_kinematics xml/hydraulics_ballcheckvalve fmi/simple_test fmi/hierachical_modelling fmi/sphere_on_plane mechanics/basics/hierachical_modelling mechanics/basics/time_dependent_kinematics

RET=0

#mfmf $(dirname $0)/linux64-dailyrelease-distribute.sh &> $OUTDIR/report_distribute/distribute.out
#mfmf test $? -ne 0 && RET=1

#mfmf cp $SRCDIR/dist_mbsim/mbsim-linux-shared-build-xxx.tar.bz2 $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
#mfmf cp $SRCDIR/dist_mbsim/mbsim-linux-shared-build-xxx-debug.tar.bz2 $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
#mfmf test $? -ne 0 && RET=1

if [ $RET -ne 0 ]; then
  rss 1
fi


$SCRIPTDIR/mergeFeeds.py
