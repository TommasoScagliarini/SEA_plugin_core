/* -------------------------------------------------------------------------- *
 * SeriesElasticActuator.cpp - FIXED & CLEANED VERSION                        *
 * -------------------------------------------------------------------------- */
#include "SeriesElasticActuator.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// COSTRUTTORI
//==============================================================================
SeriesElasticActuator::SeriesElasticActuator() {
    setAuthors("Tommaso Scagliarini");
    setReferences("Series Elastic Actuator plugin for OpenSim");
    constructProperties();

    addStateVariable("motor_angle", Stage::Dynamics);
    addStateVariable("motor_speed", Stage::Dynamics);
}

SeriesElasticActuator::SeriesElasticActuator(const std::string& name, double inertia, double damping, double k) {
    constructProperties();
    setName(name);
    //set_optimal_force(optimal_force);
    set_motor_inertia(inertia);
    set_motor_damping(damping);
    set_stiffness(k);

    
    // addStateVariable("motor_angle", Stage::Dynamics);
    // addStateVariable("motor_speed", Stage::Dynamics);
}

//==============================================================================
// COSTRUZIONE PROPRIETÀ
//==============================================================================
void SeriesElasticActuator::constructProperties() {
    constructProperty_motor_inertia(0.01);  
    constructProperty_motor_damping(0.1); 
    constructProperty_stiffness(250.0);
    constructProperty_optimal_force(100.0); 
    
    // constructProperty_min_control(-1.0);
    // constructProperty_max_control(1.0);
}

//==============================================================================
// 1. REGISTRAZIONE NEL SISTEMA 
//==============================================================================
void SeriesElasticActuator::extendAddToSystem(MultibodySystem& system) const {

    SeriesElasticActuator* mutableThis = const_cast<SeriesElasticActuator*>(this);
    if(mutableThis->getNumStateVariables() == 0) {
        
        mutableThis->addStateVariable("motor_angle", Stage::Dynamics);
        mutableThis->addStateVariable("motor_speed", Stage::Dynamics);
    }

    Super::extendAddToSystem(system);
}

//==============================================================================
// 2. CONNESSIONE MODELLO
//==============================================================================
void SeriesElasticActuator::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    // Safety check socket connection
    if(!getSocket("coordinate").isConnected()) {
        std::cerr << "[SEA WARNING] Socket 'coordinate' is not connected!" << std::endl;
    }
}

//==============================================================================
// 3. INIZIALIZZAZIONE DEGLI STATI
//==============================================================================
void SeriesElasticActuator::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    
    if(getSocket("coordinate").isConnected()) {
       const Coordinate& coord = getConnectee<Coordinate>("coordinate");
       // Il motore parte DALLA STESSA POSIZIONE del ginocchio -> Molla scarica
       double start_angle = coord.getValue(s);
       setStateVariableValue(s, "motor_angle", start_angle);
    } else {
        setStateVariableValue(s, "motor_angle", 0.0);
    }
    setStateVariableValue(s, "motor_speed", 0.0);
}

//==============================================================================
// 4. CALCOLO DERIVATE (DINAMICA MOTORE)
//==============================================================================
void SeriesElasticActuator::computeStateVariableDerivatives(const SimTK::State& s) const {
    
    // Get Motor's parameters
    double Jm = get_motor_inertia();
    double Bm = get_motor_damping();
    double K  = get_stiffness();
    double F_opt = getOptimalForce();
    if (Jm < 1e-9) Jm = 1e-9;

    double theta_m = 0.0; 
    double omega_m = 0.0;
    theta_m = getStateVariableValue(s, "motor_angle");
    omega_m = getStateVariableValue(s, "motor_speed");
    
    // Get coordinate's state
    const Coordinate& coord = getConnectee<Coordinate>("coordinate");
    double theta_joint = coord.getValue(s); 


    // Control input
    double u = 0.0;

    SimTK::Vector controls = getControls(s);
    if (controls.size() > 0) {
        u = controls[0];
     }

    static int k = 0; 
    k++;
    
    if (std::abs(u) > 0.001 && k % 100 == 0) {
        printf("[SEA] t=%.3f | u=%.2f | motor angle=%.2f | joint angle=%.2f\n", s.getTime(), u, theta_m, theta_joint);
       
    }

    // 5. Elastic torque computation
    double tau_spring = K * (theta_m - theta_joint);
    //double tau_spring = K * (theta_joint - theta_m);

    // 6. Motion equations
    double tau_input = u*F_opt;
    double theta_m_dot = omega_m;
    double omega_m_dot = (tau_input - tau_spring - (Bm * omega_m)) / Jm;

    // 7. Set the derivatives
    setStateVariableDerivativeValue(s, "motor_angle", theta_m_dot); 
    setStateVariableDerivativeValue(s, "motor_speed", omega_m_dot);
    //Super::computeStateVariableDerivatives(s);

}

double SeriesElasticActuator::computeActuation(const SimTK::State& s) const {
    // 1. Calcolo forza elastica standard
    double theta_m = getStateVariableValue(s, "motor_angle");
    double K = get_stiffness();
    const Coordinate& coord = getConnectee<Coordinate>("coordinate");
    double theta_joint = coord.getValue(s);
    
    double spring_force = K * (theta_m - theta_joint);
    
    double u = getControl(s); 
    double epsilon = 1e-5; 

    int return_type = 0;
    // Forza totale vista dall'ottimizzatore
    if (return_type == 0){
        return spring_force;

    } else if (return_type == 1){
        return spring_force + epsilon*u;

    } else if (return_type == 2){
        return spring_force + epsilon;

    } else if (return_type == 3){
        return u*getOptimalForce();
    }

}

void SeriesElasticActuator::computeForce(const SimTK::State& s, 
                                         SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                                         SimTK::Vector& generalizedForces) const {
    
    // 1. Calcola
    double force_to_apply = computeActuation(s);
    
    // 2. REGISTRA (Questo previene il crash 0xC0000005!)
    setActuation(s, force_to_apply); 

    // 3. Applica
    const Coordinate& coord = getConnectee<Coordinate>("coordinate");
    applyGeneralizedForce(s, coord, force_to_apply, generalizedForces);
}

double SeriesElasticActuator::getOptimalForce() const {
    return get_optimal_force(); 
}

double SeriesElasticActuator::getStress(const SimTK::State& s) const {
    // Calcola lo stress come |Forza| / ForzaOttimale
    // getActuation(s) è fornito da ScalarActuator e usa il tuo computeActuation
    double force = getActuation(s);
    double optForce = getOptimalForce();
    
    // Evitiamo divisioni per zero
    if (optForce < 1e-10) return 0.0;
    
    return std::abs(force) / optForce;
}

double SeriesElasticActuator::getSpeed(const SimTK::State& s) const {
    // La velocità "vista" dall'attuatore è la velocità della coordinata a cui è attaccato
    return getConnectee<Coordinate>("coordinate").getSpeedValue(s);
}
//==============================================================================
// 6. POTENZA
//==============================================================================
double SeriesElasticActuator::getPower(const State& s) const {
    double omega_m = 0.0;
    try {
        omega_m = getStateVariableValue(s, "motor_speed");
    } catch(...) { return 0.0; }

    double F_opt = getOptimalForce();
    double u = 0.0;

    SimTK::Vector controls = getControls(s);
    if (controls.size() > 0) {
        u = controls[0];
     }

    double tau_input = u * F_opt;
    /*if(isCacheVariableValid(s, "control")) {
         SimTK::Vector controls = getControls(s);
         if(controls.size() > 0) tau_input = controls[0];
    }*/
    return tau_input * omega_m;
}

void SeriesElasticActuator::implProduceForces(const SimTK::State& s,
                                               OpenSim::ForceConsumer& forceConsumer) const
{
    double force_to_apply = computeActuation(s);
    setActuation(s, force_to_apply);
    const Coordinate& coord = getConnectee<Coordinate>("coordinate");
    forceConsumer.consumeGeneralizedForce(s, coord, force_to_apply);
}