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

#include <config.h>
#include "utils.h"
#ifdef HAVE_UNORDERED_MAP
#  include <unordered_map>
#else
#  include <map>
#  define unordered_map map
#endif
#include <iostream>
#include <cmath>
#include <QtGui/QTreeWidgetItem>
#include <mbxmlutilstinyxml/getinstallpath.h>
#include <stdexcept>

using namespace std;

bool Utils::initialized=false;

void Utils::initialize() {
  if(initialized==true) return;
  initialized=true;

}

const QIcon& Utils::QIconCached(const QString& filename) {
  static unordered_map<string, QIcon> myIconCache;
  pair<unordered_map<string, QIcon>::iterator, bool> ins=myIconCache.insert(pair<string, QIcon>(filename.toStdString(), QIcon()));
  if(ins.second)
    return ins.first->second=QIcon(filename);
  return ins.first->second;
} 

// This routine is the same as MBSim::ObjectFactory::getNamespacePrefixMapping()
// However since MBSimGUI does not link to MBSim we can not call this function and therefore call
// "mbsimflatxml --printNamespacePrefixMapping" which print the content to console and reread it here.
map<string, string>& Utils::getMBSimNamespacePrefixMapping() {
  static bool firstRun=true;
  static map<string, string> nsprefix;
  if(firstRun) {
    vector<string> arg;
    arg.push_back(MBXMLUtils::getInstallPath()+"/bin/mbsimflatxml");
    arg.push_back("--printNamespacePrefixMapping");
    FILE *f=popen((arg[0]+" "+arg[1]).c_str(), "r");
    if(f==NULL) throw runtime_error("Unable to create piped process.");
    char ns[1024], prefix[1024], line[2048];
    while(1) {
      if(fgets(line, 2048, f)==NULL) break;
      int nrRead=sscanf(line, "%s %s", &ns, &prefix);
      if(nrRead<2)
        strcpy(prefix, "");
      nsprefix[ns]=prefix;
    }
    pclose(f);
    firstRun=false;
  }
  return nsprefix;
}

vector<vector<double> > mult(const vector<vector<double> > &A, const vector<vector<double> > &B) {
  vector<vector<double> > C(A.size());
  for(int i=0; i<A.size(); i++) {
    C[i].resize(B[0].size());
    for(int j=0; j<B[0].size(); j++) {
      for(int k=0; k<A[0].size(); k++)
        C[i][j] += A[i][k]*B[k][j];
    }
  }
  return C;
}

vector<vector<double> > BasicRotAKIx(double phi) {
  vector<vector<double> > AKI(3);
  for(int i=0; i<3; i++)
    AKI[i].resize(3);
  AKI[0][0]= 1.0;
  AKI[1][1]= cos(phi);
  AKI[2][2]= AKI[1][1];
  AKI[1][2]= sin(phi);
  AKI[2][1]=-AKI[1][2]; 
  return AKI;
}

vector<vector<double> > BasicRotAKIy(double phi) {
  vector<vector<double> > AKI(3);
  for(int i=0; i<3; i++)
    AKI[i].resize(3);
  AKI[1][1]= 1.0;
  AKI[0][0]= cos(phi);
  AKI[2][2]= AKI[0][0];
  AKI[0][2]=-sin(phi);
  AKI[2][0]=-AKI[0][2];
  return AKI; 
}

vector<vector<double> > BasicRotAKIz(double phi) {
  vector<vector<double> > AKI(3);
  for(int i=0; i<3; i++)
    AKI[i].resize(3);
  AKI[2][2]= 1.0;
  AKI[0][0]= cos(phi);
  AKI[1][1]= AKI[0][0];
  AKI[0][1]= sin(phi);
  AKI[1][0]= -AKI[0][1];
  return AKI; 
}

vector<vector<double> > BasicRotAIKx(double phi) {
  vector<vector<double> > AIK = BasicRotAKIx(-phi);
  return AIK;
}

vector<vector<double> > BasicRotAIKy(double phi) {
  vector<vector<double> > AIK = BasicRotAKIy(-phi);
  return AIK; 
}

vector<vector<double> > BasicRotAIKz(double phi) {
  vector<vector<double> > AIK = BasicRotAKIz(-phi);
  return AIK; 
}

vector<vector<double> > Cardan2AIK(const vector<vector<double> > &x) {
  vector<vector<double> > AIKx =  BasicRotAIKx(x[0][0]);
  vector<vector<double> > AIKy =  BasicRotAIKy(x[1][0]);
  vector<vector<double> > AIKz = BasicRotAIKz(x[2][0]);

  return mult(AIKx,mult(AIKy,AIKz));          //Wie im TM VI Skript
}

vector<vector<double> > AIK2Cardan(const vector<vector<double> > &AIK) { 
  vector<vector<double> > AlphaBetaGamma(3);
  for(int i=0; i<3; i++)
    AlphaBetaGamma[i].resize(1);
  AlphaBetaGamma[1][0]= asin(AIK[0][2]);
  double nenner = cos(AlphaBetaGamma[1][0]);
  if (fabs(nenner)>1e-10) {
    AlphaBetaGamma[0][0] = atan2(-AIK[1][2],AIK[2][2]);
    AlphaBetaGamma[2][0] = atan2(-AIK[0][1],AIK[0][0]);
  } else {
    AlphaBetaGamma[0][0]=0;
    AlphaBetaGamma[2][0]=atan2(AIK[1][0],AIK[1][1]);
  }
  return AlphaBetaGamma;
}

MBSimError::MBSimError(const std::string &mbsim_error_message_) : MBSimException(), mbsim_error_message(mbsim_error_message_) {}

  void MBSimError::printExceptionMessage() {
    cout << endl;
    cout << endl;
    cout << "A MBSimError was detected: " << mbsim_error_message << endl;
    cout << endl;
    cout << endl;
  }

QTreeWidgetItem* getChild(QTreeWidgetItem *parentItem, const QString &str) {
  for(int i=0; i<parentItem->childCount(); i++) {
    if(parentItem->child(i)->text(0) == str)
      return parentItem->child(i);
  }
  return 0;
}

string removeWhiteSpace(const string &str) {
  string ret = str;
  size_t found;
  found=ret.find_first_of(" ");
  while (found!=string::npos) {
    ret.erase(found,1);
    found=ret.find_first_of(" ",found);
  }
  return ret;
}

QString removeWhiteSpace(const QString &str) {
  QString ret = str;
  size_t found;
  found=ret.indexOf(" ");
  while (found!=string::npos) {
    ret.remove(found,1);
    found=ret.indexOf(" ",found);
  }
  return ret;
}



