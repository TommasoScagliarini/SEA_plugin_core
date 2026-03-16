/* -------------------------------------------------------------------------- *
 * SeriesElasticActuator.cpp                                                  *
 * Inherits from CoordinateActuator for native OpenSim gradient support.      *
 * -------------------------------------------------------------------------- */

#include "SeriesElasticActuator.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// CONSTRUCTORS
//==============================================================================

SeriesElasticActuator::SeriesElasticActuator() {
    setAuthors("Tommaso Scagliarini");
    setReferences("Series Elastic Actuator plugin for OpenSim");
    constructProperties();
}

SeriesElasticActuator::SeriesElasticActuator(const std::string& name,
                                             double inertia,
                                             double damping,
                                             double k,
                                             double Kp,
                                             double Kd,
                                             double optimal_force,
                                             bool   impedence)
{
    constructProperties();
    setName(name);
    set_motor_inertia(inertia);
    set_motor_damping(damping);
    set_stiffness(k);
    set_Kp(Kp);
    set_Kd(Kd);
    // setOptimalForce() is the CoordinateActuator native setter – use it
    // instead of set_optimal_force() which no longer exists on this class.
    setOptimalForce(optimal_force);
    set_Impedence(impedence);
}

//==============================================================================
// PROPERTY CONSTRUCTION
//==============================================================================

void SeriesElasticActuator::constructProperties() {
    constructProperty_motor_inertia(0.01);
    constructProperty_motor_damping(0.1);
    constructProperty_stiffness(250.0);
    constructProperty_Kp(1000.0);
    constructProperty_Kd(20.0);
    constructProperty_Impedence(false);
    // NOTE: do NOT call constructProperty_optimal_force – it belongs to
    //       CoordinateActuator and is already constructed by the parent.
    //       Use setOptimalForce(value) to change the default (100 N·m).
}

//==============================================================================
// 1. REGISTER IN MULTIBODY SYSTEM
//==============================================================================

void SeriesElasticActuator::extendAddToSystem(MultibodySystem& system) const {
    SeriesElasticActuator* mutableThis = const_cast<SeriesElasticActuator*>(this); 
    mutableThis->addStateVariable("motor_angle", Stage::Dynamics);
    mutableThis->addStateVariable("motor_speed", Stage::Dynamics);
    
    // Always call parent last so the coordinate actuator sets itself up
    // AFTER we have registered our extra state variables.
    Super::extendAddToSystem(system);
}

// NOTE: extendConnectToModel is private in CoordinateActuator (OpenSim 4.1)
// and cannot be overridden. The parent class handles coordinate validation.

//==============================================================================
// 3. INITIALISE STATE FROM PROPERTIES
//==============================================================================

void SeriesElasticActuator::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);

    const Coordinate* coord = getCoordinate();
    double start_angle = (coord != nullptr) ? coord->getValue(s) : 0.0;
    // Motor starts at the same angle as the joint → spring is unloaded.
    setStateVariableValue(s, "motor_angle", start_angle);
    setStateVariableValue(s, "motor_speed", 0.0);
    
}

//==============================================================================
// 4. MOTOR DYNAMICS  –  d/dt [theta_m, omega_m]
//==============================================================================

void SeriesElasticActuator::computeStateVariableDerivatives(const SimTK::State& s) const {
    double Jm    = get_motor_inertia();
    double Bm    = get_motor_damping();
    double K     = get_stiffness();
    double F_opt = getOptimalForce();   // CoordinateActuator native getter
    double Kp    = get_Kp();
    double Kd    = get_Kd();
    if (Jm < 1e-9) Jm = 1e-9;

    double theta_m = getStateVariableValue(s, "motor_angle");
    double omega_m = getStateVariableValue(s, "motor_speed");

    // Use CoordinateActuator's getCoordinate() instead of the socket.
    const Coordinate* coord = getCoordinate();
    double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;

    // External control signal (normalised, range ≈ [-1, 1])
    double u = getControl(s);

    static int debugCounter = 0;
    ++debugCounter;
    if (std::abs(u) > 0.001 && debugCounter % 100 == 0) {
        printf("[SEA] t=%.3f | u=%.4f | theta_m=%.4f | theta_j=%.4f\n",
            s.getTime(), u, theta_m, theta_joint);
    }

    double tau_input = 0.0;
    double tau_spring = K * (theta_m - theta_joint);

    if (get_Impedence()){
        double tau_ref = u * F_opt;
        double theta_m_ref = theta_joint + tau_ref / K;
        double omega_joint = (coord != nullptr) ? coord->getSpeedValue(s) : 0.0;
        double omega_m_ref = omega_joint;
        double tau_feedforward = tau_spring + (Bm * omega_m);

        tau_input = tau_feedforward + Kp * (theta_m_ref - theta_m) + Kd * (omega_m_ref - omega_m);

    }else{
        double tau_ref    = u * F_opt;

        tau_input  = Kp * (tau_ref - tau_spring) - Kd * omega_m;

    }
        const double MAX_MOTOR_TORQUE = 500.0;
        tau_input = std::max(-MAX_MOTOR_TORQUE, std::min(MAX_MOTOR_TORQUE, tau_input));

        double theta_m_dot = omega_m;
        double omega_m_dot = (tau_input - tau_spring - Bm * omega_m) / Jm;

        setStateVariableDerivativeValue(s, "motor_angle", theta_m_dot);
        setStateVariableDerivativeValue(s, "motor_speed",  omega_m_dot);
    
}

//==============================================================================
// 5. ACTUATION  (the single value CoordinateActuator::computeForce will apply)
//==============================================================================

double SeriesElasticActuator::computeActuation(const SimTK::State& s) const {
    if (get_Impedence()) {
        return getOptimalForce() * getControl(s); // Ideal actuation for impedance control
    } else {
        const Coordinate* coord = getCoordinate();
        double theta_joint = (coord != nullptr) ? coord->getValue(s) : 0.0;
        double theta_m     = getStateVariableValue(s, "motor_angle");
        return get_stiffness() * (theta_m - theta_joint);
    }
}

// NOTE: computeForce() is intentionally NOT overridden.
// CoordinateActuator::computeForce() calls computeActuation() internally,
// registers the actuation via setActuation(), and applies the generalised
// force correctly – giving OpenSim the analytic gradient for free.

//==============================================================================
// 6. ACCESSORS / UTILITIES
//==============================================================================

double SeriesElasticActuator::getSpeed(const SimTK::State& s) const {
    const Coordinate* coord = getCoordinate();
    return (coord != nullptr) ? coord->getSpeedValue(s) : 0.0;
}

double SeriesElasticActuator::getStress(const SimTK::State& s) const {
    double optForce = getOptimalForce();
    if (optForce < 1e-10) return 0.0;
    return std::abs(getActuation(s)) / optForce;
}

//==============================================================================
// 7. POWER
//==============================================================================

double SeriesElasticActuator::getPower(const SimTK::State& s) const {
    double omega_m = 0.0;
    try {
        omega_m = getStateVariableValue(s, "motor_speed");
    } catch (...) { return 0.0; }

    double u         = getControl(s);
    double tau_input = u * getOptimalForce();
    return tau_input * omega_m;
    
}