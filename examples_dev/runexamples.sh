#! /bin/sh

FAILED=""
GOOD=""
find -name "*.d" -exec rm -f {} \;

for D in $(find -maxdepth 1 -type d | grep -v "^\.$" | grep -v "^\./\."); do
  echo "RUNNING EXAMPLE $D"
  cd $D

  ERROR=1
  make clean && \
  make && \
  ./main && \
  ERROR=0

  if [ $ERROR -ne 0 ]; then
    echo "ERROR RUNNING EXMAPLE $D"
    FAILED="$FAILED\n$D"
  else
    echo "EXAMPLE $D PASSED"
    GOOD="$GOOD\n$D"
  fi

  cd ..
done

echo -e "EXAMPLES PASSED:$GOOD"
echo -e "EXAMPLES FAILED:$FAILED"
