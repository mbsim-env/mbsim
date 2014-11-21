#include <mbsim/dynamic_system_solver.h>
#include <mbsimControl/extern_signal_source.h>
#include <mbsimControl/extern_signal_sink.h>
#include <mbsim/extern_generalized_io.h>

namespace {
  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link);
}

class ValueReferenceMap {
  public:
    // map valueReference to union
    enum Type {
      GeneralizedIO_h,
      GeneralizedIO_x,
      GeneralizedIO_v,
      SignalSource,
      SignalSink
    };
    union TypeValue {
      MBSim::ExternGeneralizedIO *generalizedIO;
      MBSimControl::ExternSignalSource *signalSource;
      MBSimControl::ExternSignalSink *signalSink;
    };
    
    typedef std::vector<std::pair<Type, TypeValue> > VRMap;

    // create vrUnion from dss
    static void create(MBSim::DynamicSystemSolver *dss, std::vector<std::pair<Type, TypeValue> > &vrUnion) {
      // get all links from dss and store in vrUnion vector
      std::vector<MBSim::Link*> link;
      getAllLinks(dss, link);
      for(std::vector<MBSim::Link*>::iterator it=link.begin(); it!=link.end(); ++it) {
        TypeValue typeValue;
        MBSim::ExternGeneralizedIO *genIO=dynamic_cast<MBSim::ExternGeneralizedIO*>(*it);
        if(genIO) {
          typeValue.generalizedIO=genIO;
          vrUnion.push_back(std::make_pair(GeneralizedIO_h, typeValue));
          vrUnion.push_back(std::make_pair(GeneralizedIO_x, typeValue));
          vrUnion.push_back(std::make_pair(GeneralizedIO_v, typeValue));
        }
        MBSimControl::ExternSignalSource *sigSource=dynamic_cast<MBSimControl::ExternSignalSource*>(*it);
        if(sigSource) {
          typeValue.signalSource=sigSource;
          vrUnion.push_back(std::make_pair(SignalSource, typeValue));
        }
        MBSimControl::ExternSignalSink *sigSink=dynamic_cast<MBSimControl::ExternSignalSink*>(*it);
        if(sigSink) {
          typeValue.signalSink=sigSink;
          vrUnion.push_back(std::make_pair(SignalSink, typeValue));
        }
      }
    }
};

namespace {

  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link) {
    const std::vector<MBSim::Link*> &l=sys->getLinks();
    for(std::vector<MBSim::Link*>::const_iterator it=l.begin(); it!=l.end(); ++it)
      link.push_back(*it);
    const std::vector<MBSim::DynamicSystem*> &s=sys->getDynamicSystems();
    for(std::vector<MBSim::DynamicSystem*>::const_iterator it=s.begin(); it!=s.end(); ++it)
      getAllLinks(*it, link);
  }

}
