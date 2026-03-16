#ifndef SERIESELASTICACTUATOR_H
#define SERIESELASTICACTUATOR_H

/* -------------------------------------------------------------------------- *
 * SeriesElasticActuator.h                                                    *
 * Inherits from CoordinateActuator for native OpenSim gradient support.      *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
// CoordinateActuator is already included transitively by OpenSim/OpenSim.h

using namespace OpenSim;
using namespace SimTK;

class SeriesElasticActuator : public OpenSim::CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(SeriesElasticActuator, CoordinateActuator);

public:
    // -----------------------------------------------------------------------
    // Properties
    // NOTE: optimal_force and coordinate socket are already provided by
    //       CoordinateActuator – do NOT redeclare them here.
    // -----------------------------------------------------------------------
    OpenSim_DECLARE_PROPERTY(motor_inertia,  double, "Rotor inertia Jm [kg·m²]");
    OpenSim_DECLARE_PROPERTY(motor_damping,  double, "Viscous damping Bm [N·m·s/rad]");
    OpenSim_DECLARE_PROPERTY(stiffness,      double, "Spring stiffness K [N·m/rad]");
    OpenSim_DECLARE_PROPERTY(Kp,             double, "Inner torque-loop proportional gain");
    OpenSim_DECLARE_PROPERTY(Kd,             double, "Inner torque-loop derivative gain");
    OpenSim_DECLARE_PROPERTY(Impedence,          bool,   "If true the SEA is controlled by an impedence controller, otherwise by a PD controller");

    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------
    SeriesElasticActuator();

    SeriesElasticActuator(const std::string& name,
                          double inertia,
                          double damping,
                          double k,
                          double Kp,
                          double Kd,
                          double optimal_force,
                          bool   impedence);

    // -----------------------------------------------------------------------
    // Core OpenSim overrides
    // -----------------------------------------------------------------------

    /** Returns the spring torque (non-ideal) or u·F_opt (ideal). */
    double computeActuation(const SimTK::State& s) const override;

    // computeForce() is NOT overridden: CoordinateActuator::computeForce()
    // already calls computeActuation() and applies the generalised force
    // correctly, giving OpenSim the analytic gradient it needs.

    /** Motor dynamics: d/dt [theta_m, omega_m]. */
    void computeStateVariableDerivatives(const SimTK::State& s) const override;

    // -----------------------------------------------------------------------
    // Accessors / Utilities
    // -----------------------------------------------------------------------
    double getSpeed (const SimTK::State& s) const override;
    double getStress(const SimTK::State& s) const override;
    double getPower (const SimTK::State& s) const override;

protected:
    // -----------------------------------------------------------------------
    // OpenSim component pipeline overrides
    // -----------------------------------------------------------------------
    void extendAddToSystem            (SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s)               const override;

private:
    void constructProperties();
};

#endif // SERIESELASTICACTUATOR_H