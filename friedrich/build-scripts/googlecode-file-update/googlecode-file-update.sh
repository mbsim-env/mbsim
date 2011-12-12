#! /bin/sh

FILE=$1

TMPDIR=/tmp/google-code-upload

# check args
if [ "_$FILE" = "_" ]; then
  echo "Usage: $0 .../mbsimxml-linux-static.tar.bz2|.../mbsimxml-windows-static.zip"
  exit 1
fi
if [ ! -e $FILE ]; then
  echo "$FILE not found"
  exit 1
fi

# get inputs
echo -n "Enter Build Number: "
read BUILDNR

echo -n "Enter Google Acount Passwort for User $USER: "
STTY_ORG=$(stty -g)
stty -echo
read PASSWORD
stty $STTY_ORG

echo

# run
rm -rf $TMPDIR
mkdir -p $TMPDIR
if [ $(basename $FILE) = "mbsimxml-linux-static.tar.bz2" ]; then
  echo "* Copy $FILE to temp dir"
  cp $FILE $TMPDIR/mbsimxml-linux-static-build-$BUILDNR.tar.bz2
  echo "* Extract $FILE to temp dir"
  tar -xjf $FILE -C $TMPDIR
  echo "* Pack openmbv"
  ( cd $TMPDIR/mbsimxml/bin; tar -cjf $TMPDIR/openmbv-build-$BUILDNR.tar.bz2 openmbv; )
  echo "* Pack h5plotserie"
  ( cd $TMPDIR/mbsimxml/bin; tar -cjf $TMPDIR/h5plotserie-build-$BUILDNR.tar.bz2 h5plotserie; )
  echo "* Pack h5lsserie"
  ( cd $TMPDIR/mbsimxml/bin; tar -cjf $TMPDIR/h5lsserie-build-$BUILDNR.tar.bz2 h5lsserie; )
  echo "* Pack h5dumpserie"
  ( cd $TMPDIR/mbsimxml/bin; tar -cjf $TMPDIR/h5dumpserie-build-$BUILDNR.tar.bz2 h5dumpserie; )
  echo "* Upload h5dumpserie"
  python2 googlecode_upload.py -s "h5dumpserie-linux32-binary-release-build-$BUILDNR" -p hdf5serie -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Linux,Type-Archive" $TMPDIR/h5dumpserie-build-$BUILDNR.tar.bz2
  echo "* Upload h5lsserie"
  python2 googlecode_upload.py -s "h5lsserie-linux32-binary-release-build-$BUILDNR" -p hdf5serie -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Linux,Type-Archive" $TMPDIR/h5lsserie-build-$BUILDNR.tar.bz2
  echo "* Upload h5plotserie"
  python2 googlecode_upload.py -s "h5plotserie-linux32-binary-release-build-$BUILDNR" -p hdf5serie -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Linux,Type-Archive" $TMPDIR/h5plotserie-build-$BUILDNR.tar.bz2
  echo "* Upload openmbv"
  python2 googlecode_upload.py -s "openmbv-linux32-binary-release-build-$BUILDNR" -p openmbv -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Linux,Type-Archive" $TMPDIR/openmbv-build-$BUILDNR.tar.bz2
  echo "* Upload mbsimxml"
  python2 googlecode_upload.py -s "mbsimxml-linux32-binary-release-build-$BUILDNR" -p mbsim-env -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Linux,Type-Archive" $TMPDIR/mbsimxml-linux-static-build-$BUILDNR.tar.bz2
elif [ $(basename $FILE) = "mbsimxml-windows-static.zip" ]; then
  echo "* Copy $FILE to temp dir"
  cp $FILE $TMPDIR/mbsimxml-windows-static-build-$BUILDNR.zip
  echo "* Extract $FILE to temp dir"
  unzip $FILE -d $TMPDIR
  echo "* Pack openmbv"
  ( cd $TMPDIR/mbsimxml/bin; zip $TMPDIR/openmbv-build-$BUILDNR.zip openmbv.exe; )
  echo "* Pack h5plotserie"
  ( cd $TMPDIR/mbsimxml/bin; zip $TMPDIR/h5plotserie-build-$BUILDNR.zip h5plotserie.exe; )
  echo "* Pack h5lsserie"
  ( cd $TMPDIR/mbsimxml/bin; zip $TMPDIR/h5lsserie-build-$BUILDNR.zip h5lsserie.exe; )
  echo "* Pack h5dumpserie"
  ( cd $TMPDIR/mbsimxml/bin; zip $TMPDIR/h5dumpserie-build-$BUILDNR.zip h5dumpserie.exe; )
  echo "* Upload h5dumpserie"
  python2 googlecode_upload.py -s "h5dumpserie-windows32-binary-release-build-$BUILDNR" -p hdf5serie -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Windows,Type-Archive" $TMPDIR/h5dumpserie-build-$BUILDNR.zip
  echo "* Upload h5lsserie"
  python2 googlecode_upload.py -s "h5lsserie-windows32-binary-release-build-$BUILDNR" -p hdf5serie -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Windows,Type-Archive" $TMPDIR/h5lsserie-build-$BUILDNR.zip
  echo "* Upload h5plotserie"
  python2 googlecode_upload.py -s "h5plotserie-windows32-binary-release-build-$BUILDNR" -p hdf5serie -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Windows,Type-Archive" $TMPDIR/h5plotserie-build-$BUILDNR.zip
  echo "* Upload openmbv"
  python2 googlecode_upload.py -s "openmbv-windows32-binary-release-build-$BUILDNR" -p openmbv -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Windows,Type-Archive" $TMPDIR/openmbv-build-$BUILDNR.zip
  echo "* Upload mbsimxml"
  python2 googlecode_upload.py -s "mbsimxml-windows32-binary-release-build-$BUILDNR" -p mbsim-env -u friedrich.at.gc -w "$PASSWORD" -l "OpSys-Windows,Type-Archive" $TMPDIR/mbsimxml-windows-static-build-$BUILDNR.zip
else
  exit 1
fi
rm -rf $TMPDIR
