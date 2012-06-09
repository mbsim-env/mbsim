for i in $(grep -i "^set" TryRunResults.cmake | grep -v TRYRUN_OUTPUT | sed -re "s/^SET\( //"); do
  EXE=$(find -name "*$i.exe")
  echo $i
  wine $EXE
  echo $?
done
