#! /bin/sh

USER=friedrich.at.gc
DIRS="/home/user/MBSimLinux/dist_mbsim /home/user/MBSimLinux/dist_openmbv /home/user/MBSimWindows/dist_mbsim /home/user/MBSimWindows/dist_openmbv"



# get inputs
echo -n "Enter Build Number (format e.g. 004): "
read BUILDNR

echo -n "Enter GoogleCode SVN Passwort for User $USER: "
STTY_ORG=$(stty -g)
stty -echo
read PASSWORD
stty $STTY_ORG

echo

# run

cd $(dirname $0)
ABSSCRIPTDIR=$(pwd)

for D in $DIRS; do

  cd $D

  if [ -e mbsim-linux-shared-build-xxx.tar.bz2 ]; then
    SUMMARY="MBSim (inc. OpenMBV, HDF5Serie, ...) shared Linux build $BUILDNR"
    PROJECT=mbsim-env
    OS=Linux
    FILE=mbsim-linux-shared-build-xxx.tar.bz2
  fi
  if [ -e mbsim-windows-shared-build-xxx.zip ]; then
    SUMMARY="MBSim (inc. OpenMBV, HDF5Serie, ...) shared Windows build $BUILDNR"
    PROJECT=mbsim-env
    OS=Windows
    FILE=mbsim-windows-shared-build-xxx.zip
  fi
  if [ -e openmbv-linux-shared-build-xxx.tar.bz2 ]; then
    SUMMARY="OpenMBV shared Linux build $BUILDNR"
    PROJECT=openmbv
    OS=Linux
    FILE=openmbv-linux-shared-build-xxx.tar.bz2
  fi
  if [ -e openmbv-windows-shared-build-xxx.zip ]; then
    SUMMARY="OpenMBV shared Windows build $BUILDNR"
    PROJECT=openmbv
    OS=Windows
    FILE=openmbv-windows-shared-build-xxx.zip
  fi
  
  FILEBUILDNR=$(echo $FILE | sed -re "s/^(.*-)xxx(\..*)$/\1$BUILDNR\2/")
  ln -s $FILE $FILEBUILDNR
  
  echo "Upload $FILEBUILDNR"
  python2 $ABSSCRIPTDIR/googlecode_upload.py -s "$SUMMARY" -p "$PROJECT" -u "$USER" -w "$PASSWORD" -l "OpSys-$OS,Type-Archive" $FILEBUILDNR
  rm -f $FILEBUILDNR

done
