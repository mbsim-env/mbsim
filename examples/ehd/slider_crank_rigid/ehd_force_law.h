#include <mbsim/constitutive_laws.h>

/*!
 * \brief A force law that computes the normal force of many contact kinematics based on EHD theory
 * \author Kilian Grundl
 * \author Andreas Krinner
 * \date 12-06-2015 start of development
 */
class EHDForceLaw : public MBSim::GeneralizedForceLaw {
  public:
    /*!
     * \brief constructor
     */
    EHDForceLaw();

    /*!
     * \brief destructor
     */
    virtual ~EHDForceLaw();

    /* INHERITED INTERFACE */
    virtual bool isActive(double g, double gTol) {
      return true; //TODO
    }
    virtual bool remainsActive(double s, double sTol) {
      return true;
    }
    virtual bool isSetValued() const {
      return false;
    }
    virtual void computeSmoothForces(std::vector<std::vector<MBSim::SingleContact> > & contacts);
    /***************************************************/

    virtual void initializeUsingXML(xercesc::DOMElement *element);


};
