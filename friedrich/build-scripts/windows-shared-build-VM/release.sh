#! /bin/sh

SRCDIR=/home/user/MBSimWindows
OUTDIR=/media/mbsim-env/MBSimWindows
URL=http://www4.amm.mw.tu-muenchen.de/mbsim-env/MBSimWindows



svn update $SRCDIR/build-scripts

rss() {
DATE1=$(date +%s)
DATE2=$(date -R)
cat << EOF > $OUTDIR/report_distribute/result.rss.xml
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>Windows Release Build: Release Distribution</title>
    <link>$URL/report_distribute/distribute.out</link>
    <description>Windows Release Build: Result RSS feed of the MBSim Windows shared build/distribution</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="$URL/report_distribute/result.rss.xml" rel="self" type="application/rss+xml"/>
EOF
if [ $1 -eq 1 ]; then
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Windows Release Build: creating distribution failed</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_$DATE1</guid>
      <pubDate>$DATE2</pubDate>
    </item>
EOF
fi
cat << EOF >> $OUTDIR/report_distribute/result.rss.xml
    <item>
      <title>Windows Release Build: Dummy feed item. Just ignore it.</title>
      <link>$URL/report_distribute/distribute.out</link>
      <guid isPermaLink="false">$URL/report_distribute/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>
EOF
}

rss 0

$SRCDIR/build-scripts/daily-build/build.py --forceBuild "$@" --sourceDir $SRCDIR --prefix $SRCDIR/local --reportOutDir $OUTDIR/report --url $URL/report --disableRunExamples --buildType "Windows Release Build: " --passToConfigure --enable-shared --disable-static --build=i686-pc-linux-gnu --host=i686-w64-mingw32 --with-blas-lib-prefix=$SRCDIR/local/lib --with-qwt-inc-prefix=/usr/i686-w64-mingw32/sys-root/mingw/include/qwt --with-qwt-lib-name=qwt5 --with-mkoctfile=$SRCDIR/local/bin/mkoctfile.exe --with-boost-filesystem-lib=boost_filesystem-gcc47-1_48 --with-boost-system-lib=boost_system-gcc47-1_48 --with-windres=i686-w64-mingw32-windres  CPPFLAGS=-DSOQT_DLL PKG_CONFIG_PATH=/usr/i686-w64-mingw32/sys-root/mingw/lib/pkgconfig:$SRCDIR/local/lib/pkgconfig PYTHON_CFLAGS=-I/home/user/MBSimWindows/3rdparty/python/include PYTHON_LIBS="-L/home/user/MBSimWindows/3rdparty/python/libs -lpython27" PYTHON_BIN=/home/user/MBSimWindows/3rdparty/python/python.exe

RET=0

$SRCDIR/build-scripts/windows-shared-build-VM/distribute.sh &> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1

cp $SRCDIR/dist_mbsim/mbsim-windows-shared-build-xxx.zip $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1
cp $SRCDIR/dist_openmbv/openmbv-windows-shared-build-xxx.zip $OUTDIR/download/ &>> $OUTDIR/report_distribute/distribute.out
test $? -ne 0 && RET=1

if [ $RET -ne 0 ]; then
  rss 1
fi
