#! /bin/sh

if test "_$(cd $(dirname $0); pwd)" = "_$(pwd)"; then
  echo "Do not run from this or a child directory!"
  exit
fi

# create test dir
mkdir /tmp/testmbsimxml
cd /tmp/testmbsimxml
# extract
tar -xjf /home/user/project/mbsimxml-linux-static.tar.bz2
# rename project
mv /home/user/project /home/user/project_RENAMED



# run test
cd /tmp/testmbsimxml/mbsimxmlexamples/xml_hierachical_modelling
/tmp/testmbsimxml/mbsimxml/bin/mbsimxml --mbsimparam parameter.xml TS.mbsim.xml Integrator.mbsimint.xml



# de-rename project
mv /home/user/project_RENAMED /home/user/project
# remove test dir
rm -rf /tmp/testmbsimxml
