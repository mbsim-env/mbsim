#! /bin/sh

DISTDIR1=/tmp/mbsimdist
DISTDIR=$DISTDIR1/mbsimxml
LOCALDIR=/home/user/project/local

# del old temp dist
rm -rf $DISTDIR1
mkdir -p $DISTDIR







# copy all needed files to /tmp/mbsimdist/mbsimxml

# bin
BIN="
gif2h5 \
h52gif \
h5c++ \
h5cc \
h5copy \
h5debug \
h5diff \
h5dump \
h5dumpserie \
h5import \
h5jam \
h5ls \
h5lsserie \
h5mkgrp \
h5perf_serial \
h5plotserie \
h5redeploy \
h5repack \
h5repart \
h5stat \
h5unjam \
mbsimflatxml \
mbsimxml \
mbxmlutilspp \
openmbv \
"
mkdir $DISTDIR/bin
for F in $BIN; do
  cp $LOCALDIR/bin/$F $DISTDIR/bin
done

# share/hdf5serie
mkdir -p $DISTDIR/share/hdf5serie
cp -r $LOCALDIR/share/hdf5serie/* $DISTDIR/share/hdf5serie

# share/mbxmlutils
mkdir -p $DISTDIR/share/mbxmlutils
cp -r $LOCALDIR/share/mbxmlutils/* $DISTDIR/share/mbxmlutils

# share/octave
mkdir -p $DISTDIR/share/octave
cp -r $LOCALDIR/share/octave/* $DISTDIR/share/octave

# Examples
cd $DISTDIR1
svn checkout http://svn.berlios.de/svnroot/repos/mbsim/trunk/examples svncheckout
mkdir $DISTDIR1/mbsimxmlexamples
cp -r svncheckout/xml_* $DISTDIR1/mbsimxmlexamples
rm -rf svncheckout

# README
cat << EOF > $DISTDIR/README
INSTALL
=======

Unpack mbsimxml-linux-static.tar.bz2 to a directory of your choice.
# cd <myinstdir>
# tar -xvjf <downloaddir>/mbsimxml-linux-static.tar.bz2


RUN
===

Just run one of the programs in the directory <myinstdir>/mbsimxml/bin/*

A selection of programs found there are:


MBSim simulation:
# mbsimxml --mbsimparam <para.xml> <TS.mbsim.xml> <Integrator.mbsimint.xml>

Plot a H5 file (result of MBSim):
# h5plotserie

Visualize a MBSim simulation:
# openmbv

Dump metadata/data of H5 files:
# h5lsserie
# h5dumpserie


DOCUMENTATION
=============

The entry point for the MBSimXML html documentation is:
<myinstdir>/mbsimxml/share/mbxmlutils/doc/http__mbsim_berlios_de_MBSimXML/mbsimxml.xhtml


EXAMPLES
========

The directory <myinstdir>/mbsimexamples contains MBSim examples. To run e.g.
the xml_hierachical_modelling example do the following:
1. Change to the example dir
# cd <myinstdir>/mbsimxmlexamples/xml_hierachical_modelling
2. Run MBSimXML
# <myinstdir>/mbsimxml/bin/mbsimxml --mbsimparam parameter.xml TS.mbsim.xml Integrator.mbsimint.xml
3. Watch the plots
# <myinstdir>/mbsimxml/bin/h5plotserie (open the file TS.mbsim.h5 in the GUI)
4. Watch the visualization
# <myinstdir>/mbsimxml/bin/openmbv TS.ombv.xml
5. Build your own models and have fun!
EOF







# generate tar.bz2
cd $DISTDIR1
tar -cjf /home/user/project/mbsimxml-linux-static.tar.bz2 mbsimxml mbsimxmlexamples

# del temp dist
rm -rf $DISTDIR1
