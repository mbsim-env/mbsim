# Unix C-Shell variant of mbsiminit
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

if ( $?MBSIMPREFIX == 0 ) then
  echo "ERROR! You must set the environment variable MBSIMPREFIX first."
else
  setenv MBXMLUTILSSCHEMADIR ${MBSIMPREFIX}/share/mbxmlutils/schema
  setenv MBXMLUTILSXMLDIR ${MBSIMPREFIX}/share/mbxmlutils/xml
  setenv MBXMLUTILSOCTAVEDIR ${MBSIMPREFIX}/share/mbxmlutils/octave
  setenv HDF5SERIEDATADIR ${MBSIMPREFIX}/share
  setenv HDF5SERIEH5DUMP ${MBSIMPREFIX}/bin/h5dump
  setenv MBSIMBINDIR ${MBSIMPREFIX}/bin
  setenv MBXMLUTILSBINDIR ${MBSIMPREFIX}/bin
  setenv HDF5SERIEXSLTPROC ${MBSIMPREFIX}/bin/xsltproc
  echo ${PATH} | grep -E '(:|^)'${MBSIMPREFIX}'/bin(:|$)' >& /dev/null || setenv PATH ${MBSIMPREFIX}/bin:${PATH}
  echo ${LD_LIBRARY_PATH} | grep -E '(:|^)'${MBSIMPREFIX}'/lib(:|$)' >& /dev/null || setenv LD_LIBRARY_PATH ${MBSIMPREFIX}/lib:${LD_LIBRARY_PATH}

# extension for other software tools
  setenv OCTAVE_HOME ${MBSIMPREFIX}
  setenv KETSIMBINDIR ${MBSIMPREFIX}/bin
  setenv MDPCOSIMBINDIR ${MBSIMPREFIX}/bin
endif
