#! /bin/sh

SRCDIR=/home/user/MBSimLinux
OUTDIR=/media/mbsim-env/MBSimLinux
URL=http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimLinux



svn update $SRCDIR/build-scripts

rss() {
DATE1=$(date +%s)
DATE2=$(date -R)
cat << EOF > $OUTDIR/report_distribute/result.rss.xml
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>Linux Release Build: Release Distribution</title>
    <link>$URL/report_distribute/distribute.out</link>
    <description>Linux Release Build: Result RSS feed of the MBSim Linux shared build/distribution</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="$URL/report_distribute/result.rss.xml" rel="self" type="application/rss+xml"/>
EOF
if [ $1 -eq 1 ]; then
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Linux Release Build: creating distribution failed</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_$DATE1</guid>
      <pubDate>$DATE2</pubDate>
    </item>
EOF
fi
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Linux Release Build: Dummy feed item. Just ignore it.</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>
EOF
}

rss 0

$SRCDIR/build-scripts/daily-build/build.py --forceBuild "$@" --sourceDir $SRCDIR --prefix $SRCDIR/local --reportOutDir $OUTDIR/report --url $URL/report --buildType "Linux Release Build: " --passToConfigure --enable-shared --disable-static --with-qwt-inc-prefix=/usr/include/qwt --passToRunexamples xmlflat/hierachical_modelling xmlflat/time_dependent_kinematics xml/hierachical_modelling xml/time_dependent_kinematics

RET=0

$SRCDIR/build-scripts/linux-shared-build-VM/distribute.sh &> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1

cp $SRCDIR/dist_mbsim/mbsim-linux-shared-build-xxx.tar.bz2 $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1
cp $SRCDIR/dist_openmbv/openmbv-linux-shared-build-xxx.tar.bz2 $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1

if [ $RET -ne 0 ]; then
  rss 1
fi
