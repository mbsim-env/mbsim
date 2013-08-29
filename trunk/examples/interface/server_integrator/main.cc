#include "system.h"
#include <mbsimInterface/interfaceIntegrator.h>
#include <fmatvec/fmatvec.h>
#include <string>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

#include <sstream>

int main (int argc, char* argv[])
{

  /*
  cout << "Hallo Welt" << endl;

  ostringstream doubleString;
  size_t sizeOfDouble=sizeof(double);
  cout << "sizeOfDouble=" << sizeOfDouble << endl;
  double doubleValue=0.123456789;
  cout << "doubleValue=" << doubleValue << endl;
  double *doubleValueAdress=&doubleValue;
  cout << "doubleValueAdress=" << doubleValueAdress << endl;
  
  char *buff=(char*)doubleValueAdress;
  for (unsigned int i=0; i<sizeOfDouble; i++)
  {
    doubleString << *buff;
    *buff=(*buff)+sizeOfDouble;
  }
  //doubleString << ends;
  cout << "doubleString=>>> " << doubleString.str() << " <<<" << endl;

  char buffFullString[sizeOfDouble];
  char *buffFull=(char*)(&doubleValue);
  cout << "buffFull:" << buffFull << endl;
  *buffFullString=*buffFull;
  cout << "buffFullString=>>>" << buffFullString << " <<<" << endl;


  char buffReturn[sizeOfDouble];
  //buffReturn=doubleString.str().c_str();
  buffReturn[0]='A';
  buffReturn[sizeOfDouble-2]='Z';
  buffReturn[sizeOfDouble-1]='\0';
  cout << "buffReturn=" << buffReturn << endl;

  const char* buffReturn2=doubleString.str().c_str();
  cout << "buffReturn2: >>> " << buffReturn2 << " <<<" << endl;

  double *valueD=NULL;
  //(double*)(&buffReturn2[0]);
  valueD=(double*)(&buffReturn2[0]);
  cout << "*valueD=" << valueD << endl;
  cout << "valueD=" << *valueD << endl;
  cout << "valueD=" << &valueD << endl;


  double dV=doubleValue;

  char *p = new char[sizeof(double)+1];
  strcpy(p, (char*)(&dV));
  p[sizeof(double)]='\0';
  cout << p << endl;
  ostringstream ss;
  ss << p;
  const char *pp=ss.str().c_str();
  delete [] p;

  cout << "pp=" << pp << endl;

  double *ppp=NULL;
  ppp=(double*)(&pp[0]);
  cout << *ppp << endl;




  // belegen des Vektors a
  Vec a(3, INIT, 0);
  for (int i=0; i<a.size(); i++)
  {
    a(i)=(i+0)*doubleValue;
    for (int i=0; i<a.size(); i++)
      a(i)=a(i)*10;
  }
  cout << a << endl;

  // anlegen eines char-arrays und umkopieren
  char *myOutChar = new char[a.size()*(sizeof(double))];
  memcpy(myOutChar, (char*)(&a(0)), a.size()*(sizeof(double)));
  
  cout << "myOutChar=" << endl;
  ostringstream myOutSS;
  for (int i=0; i<int(a.size()*(sizeof(double))); i++)
  {
    cout << i << " " << myOutChar[i] << endl;
    myOutSS << myOutChar[i];
  }
  cout << endl;
  cout << myOutSS.str().length() << endl;
  cout << "myOutSS=" << myOutSS.str() << endl;
  
  string myOutSSStr=myOutSS.str();
  for (int i=0; i<a.size(); i++)
  {
    string abStr=myOutSSStr.substr(i*sizeof(double), sizeof(double));
    cout << abStr;
    const char *abc=abStr.c_str();
    cout << abc;
    cout << endl;

    double *ppp2=NULL;
    ppp2=(double*)(&abc[0]);
    cout << *ppp2 << endl;
  }
  */

  // build single modules
  System *sys = new System("TS");

  // add modules to overall dynamical system 
  sys->initialize();

  MBSimInterface::InterfaceIntegrator integrator;
  integrator.setStartTime(-0.14e-2);
  integrator.setEndTime(10.0);
  integrator.setPlotStepSize(1e-3);

  integrator.integrate(*sys);
  cout << "finished"<<endl;

  delete sys;

  return 0;
}

