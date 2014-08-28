#!/bin/bash

# check availability of install path of MBSim
if [ "$1" == "" ]; then
  echo "ERROR : Enter install path of MBSim as first parameter"
  exit
fi
# check if absolute or relative path
if [[ "$1" = /* ]]; then
  lib_path=$1
  bin_path=$1"bin/fmixmlexport"
else
  lib_path="../"$1
  bin_path="../"$1"bin/fmixmlexport"
fi
# check availability of the MBSim xml flat file
if [ "$2" == "" ]; then
  echo "WARNING : No MBSim-xml file specified as second parameter"
else
  # check if absolute or relative path
  if [[ "$2" = /* ]]; then
    xml_path=$2
  else
    xml_path="../"$2
  fi
fi
# point toward real lib and create the path string
if [ -d $1"lib64" ]; then
  lib_name=`readlink $1"lib64/libmbsimfmi.so"`
  lib_path=$lib_path"lib64"
else
  lib_name=`readlink $1"lib/libmbsimfmi.so"`
  lib_path=$lib_path"lib"
fi
# build the model description xml file in temp directory
mkdir -p tmp
cd tmp
$bin_path $xml_path 1>/dev/null
echo "Exporting of XML model description successful"
cd ../
# build up the folder (ie FMU) to be zipped
mkdir -p "MBSim_FMU"
mv "tmp/modelDescription.xml" "./MBSim_FMU"
rm -r ./tmp
cd "./MBSim_FMU"
# 32 or 64 bits ?
linux_version=`getconf LONG_BIT`
mkdir -p "binaries/linux$linux_version"

# copy all MBSim lib into FMU
#cp -d $lib_path/*.so* "binaries/linux$linux_version/"
#rsync -av $lib_path/ "binaries/linux$linux_version/"
bash ../../script/copy_dependencies.sh $lib_path/$lib_name "binaries/linux$linux_version/"
cp $lib_path/$lib_name "binaries/linux$linux_version/mbsim.so"

# change the name of fmi lib to correspond with model identifier
#mv "binaries/linux$linux_version/$lib_name" "binaries/linux$linux_version/mbsim.so"
#ln -s "libmbsimfmi.so" "binaries/linux$linux_version/mbsim.so"
#rm "binaries/linux$linux_version/libmbsimfmi"*
mkdir documentation
mkdir sources
# if xml file then copy it into ressources
if [ "$2" != "" ]; then
mkdir ressources
cp $xml_path "./ressources/"
fi
# zip the FMU keeping the symbolic links
zip -ry mbsim ./*
mv mbsim.zip ../mbsim.fmu
cd ../
rm -r "./MBSim_FMU"
echo "Packing of FMU successfull"
