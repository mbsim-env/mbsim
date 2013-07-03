#include "file_to_fmatvecstring.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

string FileTofmatvecString(const string &filename) {
  ifstream filestream(filename.c_str(), ifstream::in);

  string fmatvecString="[";
  string line;
  string::size_type maxSize=line.max_size();
  while (getline(filestream, line)) {

    int start=line.find_first_not_of(" ");
    // leere zeilen abfangen
    if (start<0)
      ;
    else {

      // tabs durch leerzeichen ersetzen
      while (int f=line.find("\t")) {
        if (f<0)
          break;
        line.replace(f, 1, " ");
      }
      
      int ende=line.find_last_not_of(" ");
      // fuehrende und schliessende leerzeichen loeschen
      line=line.substr(start, ende-start+1);

      // mehrere aufeinanderfolgende leerzeichen ersetzen
      while (int f=line.find("  ")) {
        if (f<0)
          break;
        line.erase(f, 1);
      }

      // leerzeichen durch komma ersetzen
      while (int f=line.find(" ")) {
        if (f<0)
          break;
        line.replace(f, 1, ",");
      }

      // kommentarzeilen abfangen
      if (line.compare(0,1,"#")&&line.compare(0,1,"%"))
        fmatvecString+=line+";";
    }
  } 
  fmatvecString=fmatvecString.substr(0,fmatvecString.size()-1);
  fmatvecString+="]";
  if (fmatvecString.length()>maxSize) {
    cout << "The input file is too long!" << endl;
    throw(123);
  }
  return fmatvecString;
}
