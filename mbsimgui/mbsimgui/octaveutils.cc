/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "octaveutils.h"
#include <string>
#include <mbxmlutilstinyxml/getinstallpath.h>
#include <stdlib.h>
#include <string.h>
#include "mbxmlutilstinyxml/tinyxml.h"
#include <sys/stat.h>
#include "env.h"

using namespace std;

void initializeOctave() {
  // check for environment variables (none default installation)
  string OCTAVEDIR;
  struct stat st;
  char *env;
  OCTAVEDIR=OCTAVEDIR_DEFAULT; // default: from build configuration
  if(stat(OCTAVEDIR.c_str(), &st)!=0) OCTAVEDIR=MBXMLUtils::getInstallPath()+"/share/mbxmlutils/octave"; // use rel path if build configuration dose not work
  if((env=getenv("MBSIMOCTAVEDIR"))) OCTAVEDIR=env; // overwrite with envvar if exist

  // initialize octave
  char **octave_argv=(char**)malloc(2*sizeof(char*));
  octave_argv[0]=(char*)malloc(6*sizeof(char*)); strcpy(octave_argv[0], "dummy");
  octave_argv[1]=(char*)malloc(3*sizeof(char*)); strcpy(octave_argv[1], "-q");
  octave_main(2, octave_argv, 1);
  int dummy;
  eval_string("warning('error','Octave:divide-by-zero');",true,dummy,0); // statement list
  eval_string("addpath('"+OCTAVEDIR+"');",true,dummy,0); // statement list

  // preserve whitespace and newline in TiXmlText nodes
  TiXmlBase::SetCondenseWhiteSpace(false);
}


