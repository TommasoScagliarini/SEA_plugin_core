/* CustomControl.cpp */
#include "CustomControl.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// Funzioni ausiliarie
double desiredModelZPosition( double t ) {
    return 2* sin( 1.5* t ); 
}

double desiredModelZVelocity( double t ) {
    return 1* 0.5 * cos( 0.5 * t );
}

// ============================================================================
// IMPLEMENTAZIONE
// ============================================================================

CustomController::CustomController() : Controller() {
    constructProperties();
    // RIMOSSO: constructSocket_coordinate(); NON ESISTE
}

CustomController::CustomController(double aKp, double aKv) : Controller() 
{
    constructProperties();
    // RIMOSSO: constructSocket_coordinate(); NON ESISTE
    set_kp(aKp);
    set_kv(aKv);
}

void CustomController::constructProperties() {
    constructProperty_kp(100.0);
    constructProperty_kv(20.0);
}

void CustomController::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    double t = s.getTime();
    double kp = get_kp();
    double kv = get_kv();
    
    

    const Model& model = getModel();
    double g = abs(model.getGravity()[1]);
    
    const Body& bodyLink2 = model.getBodySet().get("link2"); 
    double m2 = bodyLink2.getMass();
    
    const Coordinate& coord1 = model.getCoordinateSet().get("theta1");
    double theta1 = coord1.getValue(s);


    double pos_des = desiredModelZPosition(t);
    // double vel_des = desiredModelZVelocity(t);

    // double pos_des = SimTK::convertDegreesToRadians(90.0); // 30 gradi in radianti
    double vel_des = 0;

    const Coordinate& coord = getConnectee<Coordinate>("coordinate");

    double z  = coord.getValue(s);
    double zv = coord.getSpeedValue(s);

    double maxInput = 20, minInput = -maxInput;
    double gravityCompensation = m2* g * 0.5 * sin(theta1 + z); // Aggiustare se necessario
    
    double u = kp * (pos_des - z) + kv * (vel_des - zv) + gravityCompensation; // PD + gravity compensation

    if (u > maxInput) {
        u = maxInput;
    }else if (u < minInput) {
        u = minInput;
    }

    
    const Set<const Actuator>& actuatorSet = getActuatorSet();
    
    if(actuatorSet.getSize() > 0) {
        // Prendo il primo attuatore e aggiungo il controllo
        const Actuator& act = actuatorSet.get(0);
        SimTK::Vector actuatorControl(1, u);
        act.addInControls(actuatorControl, controls);
    }
}