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
  # setup link
  rm -f /tmp/mbsim_local
  test -L /tmp/mbsim_local || ln -s $MBSIMPREFIX /tmp/mbsim_local
  # setup PATH and LD_LIBRARY_PATH
  mbsimflatxml -h >& /dev/null || setenv LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:/tmp/mbsim_local/lib
  mbsimflatxml -h >& /dev/null || setenv PATH ${PATH}:/tmp/mbsim_local/bin
endif
