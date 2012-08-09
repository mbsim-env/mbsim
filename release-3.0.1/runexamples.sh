#! /bin/sh

if [ $# -eq 1 -a "$1" = "-h" ]; then
  echo "Usage:"
  echo ""
  echo "This script must be executed in the current directory!"
  echo ""
  echo "Two dataset differ, if at least one data has a relative AND absoulute"
  echo "tolerance higher than RTOL and ATOL respectively!"
  echo ""
  echo "runexamples.sh (run all examples with default RTOL=1e-6 and ATOL=1e-6)"
  echo "runexamples.sh 1e-3,1e-4 (run all examples with RTOL=1e-3 and ATOL=1e-4)"
  echo "runexamples.sh 2/3 (group all examples in blocks of 3 and run the 2 of each block;"
  echo "                    So running 1/3 in one console and 2/3 and 3/3 in another"
  echo "                    console will run all examples in parallel)"
  echo "runexamples.sh robot (run robot example with default RTOL=1e-6 and ATOL=1e-6)"
  echo "runexamples.sh robot 1e-3,1e-4 (run robot example with RTOL=1e-3 AND ATOL=1e-4)"
  echo "runexamples.sh dist (make a ./reference.tar.bz2 reference distribution using"
  echo "        the data stored in the reference directory of the respective example)"
  echo "runexamples.sh pushdist (push the reference file ./reference.tar.bz2 to the"
  echo "                         berlios mbsim ftp server)"
  echo "runexamples.sh install (install the reference.tar.bz2 reference file from"
  echo "                        the berlios mbsim ftp server)"
  echo "runexamples.sh install ../myref/ref.tar.bz2 (install the ../myref/ref.tar.bz2"
  echo "                                            reference file)"
  echo "runexamples.sh validateXML (validate all *.mbsim.xml and *.ombv.xml files)"
  echo "runexamples.sh [xmlflat_|xml_|xml] ...   run only example dir starting with"
  exit
fi

# which examples to run
WHICHEXAMPLES=""
if [ "_$1" == "_xmlflat_" -o "_$1" == "_xml" -o "_$1" == "_xml_" ]; then
  WHICHEXAMPLES=$1
  shift
fi

RTOL=1e-6
ATOL=1e-6
EXAMPLES=$(find -maxdepth 1 -type d | grep -v "^\.$" | grep -v "^\./\." | grep "^\./$WHICHEXAMPLES")
if [ $# -eq 1 ]; then
  if echo $1 | grep -E "^[0-9]+/[0-9]+$" > /dev/null; then
    I=$(echo $1 | sed -re "s|^([0-9]+)/.*$|\1|")
    N=$(echo $1 | sed -re "s|^.*/([0-9]+)$|\1|")
    i=1
    PARTIALEXAMPLES=""
    for E in $EXAMPLES; do
      if [ $i -eq $I ]; then
        PARTIALEXAMPLES="$PARTIALEXAMPLES $E"
        I=$[$I+$N]
      fi
      i=$[$i+1]
    done
    EXAMPLES=$PARTIALEXAMPLES
  fi
  if [ "$1" = "dist" ]; then
    echo "Making a distribution of reference files: reference.tar.bz2"
    tar -cvjf reference.tar.bz2 $(find -maxdepth 2 -name "reference")
    exit
  fi
  if [ "$1" = "pushdist" ]; then
    echo "Pushing the reference file to the mbsim ftp server."
    echo -n "Username@shell.berlios.de: "
    read U
    cat reference.tar.bz2 | ssh $U@shell.berlios.de "cat - > /home/groups/ftp/pub/mbsim/reference.tar.bz2; chmod g+w /home/groups/ftp/pub/mbsim/reference.tar.bz2"
    exit
  fi
  if [ "$1" = "install" ]; then
    echo "Download reference file"
    rm reference.tar.bz2
    wget ftp://ftp.berlios.de/pub/mbsim/reference.tar.bz2
    echo "Install the reference file"
    tar -xvjf reference.tar.bz2
    exit
  fi
  XMLLINT=xmllint
  test -f $(pkg-config --variable=BINDIR mbxmlutils)/xmllint && XMLLINT=$(pkg-config --variable=BINDIR mbxmlutils)/xmllint
  if [ "$1" = "validateXML" ]; then
    find -maxdepth 2 -name "*.ombv.xml" | xargs $XMLLINT --xinclude --noout --schema $(pkg-config --variable SCHEMADIR mbxmlutils)/http___openmbv_berlios_de_OpenMBV/openmbv.xsd 2>&1 | grep -v " validates$"
    find -maxdepth 2 -name "*.ombv.env.xml" | xargs $XMLLINT --xinclude --noout --schema $(pkg-config --variable SCHEMADIR mbxmlutils)/http___openmbv_berlios_de_OpenMBV/openmbv.xsd 2>&1 | grep -v " validates$"
    find -maxdepth 2 -name "*.mbsim.xml" | grep -v ".*/\." | xargs $XMLLINT --xinclude --noout --schema $(pkg-config --variable SCHEMADIR mbxmlutils)/http___mbsim_berlios_de_MBSimXML/mbsimxml.xsd 2>&1 | grep -v " validates$"
    find -maxdepth 2 -name "*.mbsimint.xml" | grep -v ".*/\." | xargs $XMLLINT --xinclude --noout --schema $(pkg-config --variable SCHEMADIR mbxmlutils)/http___mbsim_berlios_de_MBSim/mbsimintegrator.xsd 2>&1 | grep -v " validates$"
    exit
  fi
  if cd $1 &> /dev/null; then
    EXAMPLES=$1
    cd ..
  else
    RTOL=$(echo $1 | sed -re "s/^([^,]+),([^,]+)$/\1/")
    ATOL=$(echo $1 | sed -re "s/^([^,]+),([^,]+)$/\2/")
  fi
fi
if [ $# -eq 2 ]; then
  if [ "$1" = "install" ]; then
    echo "Install the reference file: $2"
    tar -xvjf $2
    exit
  fi
  EXAMPLES=$1
  RTOL=$(echo $2 | sed -re "s/^([^,]+),([^,]+)$/\1/")
  ATOL=$(echo $2 | sed -re "s/^([^,]+),([^,]+)$/\2/")
fi


# RUN EXAMPLES

FAILED=""
DIFF=""
find -name "*.d" -exec rm -f {} \;

NR=0
NRMAX=$(echo $EXAMPLES | wc -w)
TIMEFILE=$(pwd)/runexamples.time
rm $TIMEFILE
for D in $EXAMPLES; do
  NR=$[$NR+1]
  echo -ne "\033]0;RUNNING EXAMPLE $NR/$NRMAX $D\007"
  echo "RUNNING EXAMPLE $NR/$NRMAX $D"
  cd $D

  XMLEXAMPLE=false
  echo $D | grep "^\./xmlflat_" && XMLEXAMPLE=true
  echo $D | grep "^xmlflat_" && XMLEXAMPLE=true
  echo $D | grep "^\./xml_" && XMLEXAMPLE=true_pp
  echo $D | grep "^xml_" && XMLEXAMPLE=true_pp

  EXTRAARGS=""
  STARTTIME=$(date +%s)
  if [ $XMLEXAMPLE == false ]; then
    ERROR=1
    make clean && \
    make && \
    ./main && \
    ERROR=0
  elif [ $XMLEXAMPLE == true ]; then
    ERROR=1
    $(pkg-config mbsim --variable=bindir)/mbsimflatxml TS.mbsim.xml Integrator.mbsimint.xml && ERROR=0
  else
    ERROR=1
    if [ -f parameter.xml ]; then
      EXTRAARGS="$EXTRAARGS --mbsimparam parameter.xml"
    fi
    if [ -d mfiles ]; then
      EXTRAARGS="$EXTRAARGS --mpath mfiles"
    fi
    $(pkg-config mbsim --variable=bindir)/mbsimxml $EXTRAARGS TS.mbsim.xml Integrator.mbsimint.xml && ERROR=0
  fi
  ENDTIME=$(date +%s)
  echo "$D: $[$ENDTIME-$STARTTIME]" >> $TIMEFILE

  if [ $ERROR -eq 0 ]; then
    echo "EXAMPLE $D PASSED COMPILING AND RUNNING"
  else
    echo "EXAMPLE $D FAILED COMPILING OR RUNNING"
    FAILED="$FAILED\n$D"
  fi

  H5DIFF=h5diff
  test -f $(pkg-config hdf5serie --variable=hdf5_prefix)/bin/h5diff && H5DIFF=$(pkg-config hdf5serie --variable=hdf5_prefix)/bin/h5diff
  if test -d reference -a $ERROR -eq 0; then
    for H5F in $(cd reference && find -name "*.h5"); do
      for DS in $($(pkg-config hdf5serie --variable=bindir)/h5lsserie reference/$H5F | sed -nre "s|^.*\(Path: \"(.*)\"\)$|\1|p"); do
        P=$(echo $DS | sed -re "s|^.*\.h5/(.*)|\1|")
        $H5DIFF --nan -p=$RTOL -d=$ATOL "$H5F" "reference/$H5F" "$P" "$P"
        RET=$?
        if [ $RET -ne 0 ]; then
          echo "EXAMPLE $DS FAILED DIFF WITH REFERENCE SOLUTION"
          DIFF="$DIFF\n$D/$DS"
        else
          echo "EXAMPLE $DS PASSED DIFF WITH REFERENCE SOLUTION"
        fi
      done
    done
  fi

  cd ..
done

echo -e "\n\n\n\n\n\n\n\n\n\n"
echo -e "EXAMPLES FAILED COMPILING OR RUNNING:$FAILED"
echo -e "\n"
echo -e "EXAMPLES FAILED DIFF WITH REFERENCE SOLUTION:$DIFF"



echo -e "\n\n\n\n\n\n\n\n\n\n" > runexamples.log
echo -e "EXAMPLES FAILED COMPILING OR RUNNING:$FAILED" >> runexamples.log
echo -e "\n" >> runexamples.log
echo -e "EXAMPLES FAILED DIFF WITH REFERENCE SOLUTION:$DIFF" >> runexamples.log

# return
if [ "_$FAILED" != "_" -o "_$DIFF" != "_" ]; then
  exit 1 # return with error
fi
exit 0 # return without error
