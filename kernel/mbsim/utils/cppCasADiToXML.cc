#include <config.h>
#include <cassert>
#include <cfenv>
#include <mbxmlutilshelper/casadiXML.h>
#include <sstream>
#include <casadi/core/function/function.hpp>

using namespace std;
using namespace casadi;
using namespace MBXMLUtils;
using namespace xercesc;

int main() {
#ifndef _WIN32
  assert(feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)!=-1);
#endif

  // the rotation symbolic equation using by xmlflat/time_dependent_kinematics
  double freq2=M_PI/3;
  SX t=SX::sym("t");
  SX fexp=5*sin(freq2*t);

  shared_ptr<DOMParser> parser=DOMParser::create();
  shared_ptr<DOMDocument> xmlFile=parser->createDocument();

  DOMElement *ele=convertCasADiToXML({{t}, {fexp}},xmlFile.get());
  xmlFile->insertBefore(ele, NULL);

  cout<<"Output XML representation to out.xml"<<endl<<endl;
  DOMParser::serialize(xmlFile.get(), "out.xml");

  cout<<endl<<"Reread XML and print original and reread as CasADi stream"<<endl<<endl;
  auto fReread=createCasADiFunctionFromXML(xmlFile->getDocumentElement());
  auto tReread=fReread.first;
  auto fexpReread=fReread.second;
  {
    stringstream fStr; fStr<<t;
    stringstream fRereadStr; fRereadStr<<tReread;
    cout<<"original input  : "<<fStr.str()<<endl;
    cout<<"reread   input  : "<<fRereadStr.str()<<endl;
    if(fStr.str()!=fRereadStr.str())
      return 1;
  }
  {
    stringstream fStr; fStr<<fexp;
    stringstream fRereadStr; fRereadStr<<fexpReread;
    cout<<"original output : "<<fStr.str()<<endl;
    cout<<"reread   output : "<<fRereadStr.str()<<endl;
    if(fStr.str()!=fRereadStr.str())
      return 1;
  }


  cout<<endl<<"Evaluate function"<<endl<<endl;

  vector<DM> arg{3.546}, ret;
  Function f("noname", {t}, {fexp});
  ret=f(arg);
  cout<<"Arg: "<<arg[0].scalar()<<endl;
  cout<<"Ret: "<<ret[0].scalar()<<endl;
  cout<<"Ref: "<<5*sin(freq2*arg[0].scalar())<<endl;


  cout<<endl<<"Evaluate jacobian"<<endl<<endl;

  Function j=Function("noname", {t}, {jacobian(fexp, t)});
  cout<<"Arg: "<<arg[0].scalar()<<endl;
  cout<<"Ret: "<<j(arg)[0].scalar()<<endl;
  cout<<"Ref: "<<freq2*5*cos(freq2*arg[0].scalar())<<endl;


  cout<<endl<<"Evaluate directional derivative"<<endl<<endl;

  SX td=SX::sym("td");
  DM argd(6.424);
  Function dd=Function("noname", {td, t}, {jtimes(fexp, t, td)});
  cout<<"Arg: "<<arg[0].scalar()<<endl;
  cout<<"Argd: "<<argd.scalar()<<endl;
  cout<<"Ret: "<<dd(vector<SX>{argd, arg[0]})[0].scalar()<<endl;
  cout<<"Ref: "<<freq2*5*cos(freq2*arg[0].scalar())*argd.scalar()<<endl;

  return 0;
}
