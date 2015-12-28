for i in $(sed -rne "s/\<SET *\( *(.*)/\1/"p TryRunResults.cmake | grep -v __TRYRUN_OUTPUT); do
  EXE=$(find -name "*$i.exe")
  echo
  echo
  echo "START $EXE"
  wine $EXE
  echo "RET $?"
done
