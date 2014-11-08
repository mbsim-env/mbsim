#!/bin/bash
BIT=$(getconf LONG_BIT)
rm *.tmp
echo "Start 1st test"
../script/mbsimfmi.sh ../../../../MBSim/mbsim/examples/xmlflat/bouncing_ball/MBS.mbsimprj.flat.xml 1>/dev/null
../extern/fmuCheck.linux$BIT mbsim.fmu >out_bouncing.tmp 2>&1 #1>&-
mv mbsim.fmu mbsimB.fmu
echo "Clean folder"
#rm ./*.h5
#rm ./*.xml
echo "Script completed"
