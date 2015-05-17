#include <config.h>
#include <mbxmlutilshelper/casadiXML.h>

using namespace std;
using namespace boost;
using namespace casadi;
using namespace MBXMLUtils;
using namespace xercesc;

int main() {
//  // test SXFunction you want to export as XML
//  vector<SX> input;
//  SX t=SX::sym("t"); // a scalar input
//  input.push_back(t);
//  SX q=casadi::ssym("q", 2, 1); // a column vector input
//  input.push_back(q);
//  SX qd=casadi::ssym("qd", 1, 2); // a row vector input
//  input.push_back(qd);
//  SX J=casadi::ssym("J", 2, 2); // a matrix input
//  input.push_back(J);
//  SX r=q.elem(0)*q.elem(1)+sin(5*t)+t*t; // a scalar output
//  SXFunction f(input, r); // the function

//  // the translation symbolic equation using by xmlflat/time_dependent_kinematics
//  double freq1 = M_PI;
//  double v0y = 1;
//  SX t=SX::sym("t");
//  SX fexp=SX::zeros(3);
//  fexp[0] = sin(freq1*t + M_PI/2);
//  fexp[1] = v0y*t; 
//  fexp[2] = 0; 
//  SXFunction f(t,fexp);

  // the rotation symbolic equation using by xmlflat/time_dependent_kinematics
  double freq2=M_PI/3;
  SX t=SX::sym("t");
  SX fexp=5*sin(freq2*t);
  SXFunction f(t,fexp);

  shared_ptr<DOMParser> parser=DOMParser::create(false);
  shared_ptr<DOMDocument> xmlFile=parser->createDocument();

  DOMElement *ele=convertCasADiToXML(f,xmlFile.get());
  xmlFile->insertBefore(ele, NULL);

  cout << E(ele)->getTagName().second << endl;

  cout<<"XML representation"<<endl<<endl;
  DOMParser::serialize(xmlFile.get(), "out.xml");

  cout<<endl<<"Reread XML and print original and reread as CasADi stream"<<endl<<endl;
  casadi::SXFunction fReread=createCasADiSXFunctionFromXML(xmlFile->getDocumentElement());
  fReread.init();
  fReread.evaluate();
  for(int i=0; i<f.inputExpr().size(); i++) {
    cout<<"original input  "<<i<<": "<<f.inputExpr(i)<<endl;
    cout<<"reread   input  "<<i<<": "<<fReread.inputExpr(i)<<endl;
  }
  for(int i=0; i<f.outputExpr().size(); i++) {
    cout<<"original output "<<i<<": "<<f.outputExpr(i)<<endl;
    cout<<"reread   output "<<i<<": "<<fReread.outputExpr(i)<<endl;
  }

  return 0;
}
