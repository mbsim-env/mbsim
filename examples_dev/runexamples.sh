#! /bin/sh

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
        ../../local/bin/h5diff --relative=1e-6 $H5F reference/$H5F $P $P
        RET=$?
        if [ $RET -ne 0 ]; then
          echo "EXAMPLE $DS FAILED DIFF WITH REFERENCE"
          DIFF="$DIFF\n$D/$DS"
        else
          echo "EXAMPLE $DS PASSED DIFF WITH REFERENCE"
        fi
      done
    done
  fi

  cd ..
done

echo -e "\n\n\n\n\n\n\n\n\n\n"
echo -e "EXAMPLES FAILED COMPINLING OR RUNNING:$FAILED"
echo -e "\n"
echo -e "EXAMPLES FAILED DIFF:$DIFF"
