#!/bin/bash

set -e

prefix=$(cd $(dirname $(dirname $0)); pwd)
lib_path=$prefix
bin_path=$prefix/bin/fmixmlexport
# check availability of the MBSim xml flat file
if [ "$1" == "" ]; then
  echo "WARNING : No MBSim-xml file specified as second parameter"
else
  # check if absolute or relative path
  if [[ "$1" = /* ]]; then
    xml_path=$1
  else
    xml_path="../"$1
  fi
fi
# point toward real lib and create the path string
if [ -d $prefix/lib64 ]; then
  lib_name=`readlink $prefix/lib64/libmbsimfmi.so`
  lib_path=$lib_path/lib64
else
  lib_name=`readlink $prefix/lib/libmbsimfmi.so`
  lib_path=$lib_path/lib
fi
# build the model description xml file in temp directory
rm -rf ./tmp
mkdir tmp
cd tmp
$bin_path $xml_path
echo "Exporting of XML model description successful"
cd ../
# build up the folder (ie FMU) to be zipped
rm -rf "./MBSim_FMU"
mkdir "MBSim_FMU"
mv "tmp/modelDescription.xml" "./MBSim_FMU"
rm -r ./tmp
cd "./MBSim_FMU"
# 32 or 64 bits ?
linux_version=`getconf LONG_BIT`
mkdir -p "binaries/linux$linux_version"

# copy all MBSim lib into FMU
#cp -d $lib_path/*.so* "binaries/linux$linux_version/"
#rsync -av $lib_path/ "binaries/linux$linux_version/"
$prefix/bin/mbsimfmi_copy_dependencies.sh $lib_path/$lib_name "binaries/linux$linux_version/"
cp $lib_path/$lib_name "binaries/linux$linux_version/mbsim.so"

# change the name of fmi lib to correspond with model identifier
#mv "binaries/linux$linux_version/$lib_name" "binaries/linux$linux_version/mbsim.so"
#ln -s "libmbsimfmi.so" "binaries/linux$linux_version/mbsim.so"
#rm "binaries/linux$linux_version/libmbsimfmi"*
mkdir documentation
mkdir sources
# if xml file then copy it into resources
if [ "$1" != "" ]; then
mkdir resources
cp $xml_path "./resources/"
fi
# zip the FMU keeping the symbolic links
zip -ry mbsim ./*
mv mbsim.zip ../mbsim.fmu
cd ../
rm -r "./MBSim_FMU"
echo "Packing of FMU successfull"
