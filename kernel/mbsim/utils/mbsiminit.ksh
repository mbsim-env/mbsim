# Unix Korn-Shell variant of mbsiminit
# DEVELOPER: If editing this file, edit also the files mbsiminit.*!
#
# Copyright (C) 2009 Markus Friedrich
#
# This library is free software; you can redistribute it and/or 
# modify it under the terms of the GNU Lesser General Public 
# License as published by the Free Software Foundation; either 
# version 2.1 of the License, or (at your option) any later version. 
#  
# This library is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
# Lesser General Public License for more details. 
#  
# You should have received a copy of the GNU Lesser General Public 
# License along with this library; if not, write to the Free Software 
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
#
# Contact:
#   mafriedrich@users.berlios.de

if test -z $MBSIMPREFIX; then
  echo "ERROR! You must set the environment variable MBSIMPREFIX first."
else
  export MBXMLUTILSSCHEMADIR=${MBSIMPREFIX}/share/mbxmlutils/schema
  export MBXMLUTILSXMLDIR=${MBSIMPREFIX}/share/mbxmlutils/xml
  export MBXMLUTILSOCTAVEDIR=${MBSIMPREFIX}/share/mbxmlutils/octave
  export MBXMLUTILSOCTAVEPREFIX=${MBSIMPREFIX}
  export HDF5SERIEDATADIR=${MBSIMPREFIX}/share
  export HDF5SERIEH5DUMP=${MBSIMPREFIX}/bin/h5dump
  export MBSIMBINDIR=${MBSIMPREFIX}/bin
  export MBXMLUTILSBINDIR=${MBSIMPREFIX}/bin
  export HDF5SERIEXSLTPROC=${MBSIMPREFIX}/bin/xsltproc
  mbsimflatxml -h > /dev/null 2> /dev/null || export PATH=$PATH:${MBSIMPREFIX}/bin
  mbsimflatxml -h > /dev/null 2> /dev/null || export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MBSIMPREFIX}/lib

# extension for other software tools
  export KETSIMBINDIR=${MBSIMPREFIX}/bin
  export MDPCOSIMBINDIR=${MBSIMPREFIX}/bin
fi
