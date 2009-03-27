#! /bin/sh

RTOL=1e-6
if [ $# -gt 0 ]; then
  if [ $1 = "dist" ]; then
    echo "Making a distribution of reference files: reference.tar.bz2"
    tar -cjf reference.tar.bz2 $(find -name "reference")
    exit
  elif [ $1 = "install" -a $# -eq 2 ]; then
    echo "Install the reference file: $2"
    tar -xjf $2
    exit
  else
    RTOL=$1
  fi
fi

FAILED=""
DIFF=""
find -name "*.d" -exec rm -f {} \;

for D in $(find -maxdepth 1 -type d | grep -v "^\.$" | grep -v "^\./\."); do
  echo "RUNNING EXAMPLE $D"
  cd $D

  ERROR=1
  make clean && \
  make && \
  ./main && \
  ERROR=0

  if [ $ERROR -eq 0 ]; then
    echo "EXAMPLE $D PASSED COMPILING AND RUNNING"
  else
    echo "EXAMPLE $D FAILED COMPILING OR RUNNING"
    FAILED="$FAILED\n$D"
  fi

  if test -d reference; then
    for H5F in $(cd reference && find -name "*.h5"); do
      for DS in $(../../local/bin/h5lsserie $H5F | sed -nre "s|^.*\(Path: (.*)\)|\1|p"); do
        P=$(echo $DS | sed -re "s|^.*\.h5/(.*)|\1|")
        ../../local/bin/h5diff --relative=$RTOL $H5F reference/$H5F $P $P
        RET=$?
        if [ $RET -ne 0 ]; then
          echo "EXAMPLE $DS FAILED DIFF WITH REFERENCE SOULUTION"
          DIFF="$DIFF\n$D/$DS"
        else
          echo "EXAMPLE $DS PASSED DIFF WITH REFERENCE SOULUTION"
        fi
      done
    done
  fi

  cd ..
done

echo -e "\n\n\n\n\n\n\n\n\n\n"
echo -e "EXAMPLES FAILED COMPILING OR RUNNING:$FAILED"
echo -e "\n"
echo -e "EXAMPLES FAILED DIFF WITH REFERENCE SOULUTION:$DIFF"
