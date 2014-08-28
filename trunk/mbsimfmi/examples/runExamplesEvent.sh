#!/bin/bash
rm *.tmp
echo "Start 1st test"
bash ../script/pack_FMU.sh ../../../../MBSim/Install/ ../../../../MBSim/mbsim/examples/xmlflat/tippe_top/MBS.mbsimprj.flat.xml 1>/dev/null
./fmuCheck.linux64 mbsim.fmu >out_tippe.tmp 2>&1 #1>&-
mv mbsim.fmu mbsimT.fmu
echo "Clean folder"
#rm ./*.h5
#rm ./*.xml
echo "Script completed"
