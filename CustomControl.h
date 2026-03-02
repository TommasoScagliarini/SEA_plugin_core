/* CustomControl.h */
#ifndef OPENSIM_CUSTOM_CONTROLLER_H_
#define OPENSIM_CUSTOM_CONTROLLER_H_

#include <OpenSim/OpenSim.h>

namespace OpenSim {

class CustomController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(CustomController, Controller);
    
public: // <--- SPOSTA "public:" QUI SOPRA
    
    // Ora il socket è pubblico e accessibile dal main
    OpenSim_DECLARE_SOCKET(coordinate, Coordinate, "Coordinata da controllare");

    OpenSim_DECLARE_PROPERTY(kp, double, "Guadagno proporzionale");
    OpenSim_DECLARE_PROPERTY(kv, double, "Guadagno derivativo");

    CustomController();
    CustomController(double aKp, double aKv);

    void computeControls(const SimTK::State& s, SimTK::Vector& controls) const override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif