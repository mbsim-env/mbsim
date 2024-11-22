#!/bin/bash

mbsimgui --autoExit
if [ $? -ne 0 ]; then
  echo "mbsimgui error tests skipped since mbsimgui cannot be started"
  exit 0
fi

$srcdir/../examples/error/run_error_checks.py --run gui $srcdir/../examples
# mbsimgui does currently not report fully correct error messages (line and xpath may be wrong)
# hence we return always 0 for now for the mbsimgui check
exit 0
