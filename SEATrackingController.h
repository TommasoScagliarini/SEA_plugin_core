#ifndef SEATRACKINGCONTROLLER_H
#define SEATRACKINGCONTROLLER_H

#include <OpenSim/OpenSim.h>
#include "SeriesElasticActuator.h"

using namespace OpenSim;
using namespace SimTK;

class SEATrackingController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(SEATrackingController, Controller);

public:
    // Costruttore
    SEATrackingController(const std::string& forcesFileName, const std::string& actuatorName, const std::string& seaName, double Kp, double Kd) 
        : _actuatorName(actuatorName), _seaName(seaName), _Kp(Kp), _Kd(Kd) 
    {
        // 1. Carica il file delle forze ottimali calcolate dal CMC
        Storage tauDesStorage(forcesFileName);
        
        // 2. Trova l'indice della colonna dell'attuatore ideale usato nel CMC
        int colIndex = tauDesStorage.getStateIndex(actuatorName);
        if (colIndex < 0) throw OpenSim::Exception("Attuatore non trovato nel file delle forze!");

        // 3. Estrai i dati di tempo e forza
        int nRows = tauDesStorage.getSize();
        Array<double> times(0.0, nRows);
        Array<double> forces(0.0, nRows);
        tauDesStorage.getTimeColumn(times);
        tauDesStorage.getDataColumn(colIndex, forces);

        // 4. Generate a Poly5 spline intarpolating the desired torque over time
        _tauDesFunc = new GCVSpline(5, nRows, &times[0], &forces[0]);
    }

    ~SEATrackingController() {
        delete _tauDesFunc;
    }

    // Questa funzione viene chiamata ad OGNI frame della simulazione
    void computeControls(const State& s, Vector& controls) const override {
        double t = s.getTime();
        Vector timeVec(1, t);
        double tau_des = _tauDesFunc->calcValue(timeVec);
        
        // B. Trova il nostro SEA
        auto sea = dynamic_cast<const SeriesElasticActuator*>(&getModel().getComponent<Actuator>("forceset/SEA"));//(_seaName));
        if (!sea) return;

        // C. Leggi gli stati attuali del SEA
        double theta_m = sea->getStateVariableValue(s, "motor_angle");
        double omega_m = sea->getStateVariableValue(s, "motor_speed");
        
        // Calcola la forza attuale della molla: K * (theta_m - theta_joint)
        double K = sea->get_stiffness();
        double theta_joint = sea->getConnectee<Coordinate>("coordinate").getValue(s);
        double tau_spring = K * (theta_m - theta_joint);

        // D. Calcola l'errore e il segnale di controllo (Legge PD)
        double error = tau_des - tau_spring;
        double u = 0;

        int choice = 2;
        if (choice == 0) {
            u = (_Kp * error) - (_Kd * omega_m);
        } else if (choice == 1){
            u = (_Kp * error);
        }else if (choice == 2){
            std::vector<int> derivComponents = {0};
            double tau_des_dot = _tauDesFunc->calcDerivative(derivComponents, timeVec); 
            double omega_joint = sea->getConnectee<Coordinate>("coordinate").getSpeedValue(s);
            double error_dot = tau_des_dot - K*(omega_m - omega_joint);
            u = _Kp * error + _Kd * (error_dot);
        }
        

        // E. Applica il controllo al motore!
        Vector myControl(1, u);
        sea->addInControls(myControl, controls);
    }

private:
    std::string _actuatorName;
    std::string _seaName;
    double _Kp;
    double _Kd;
    GCVSpline* _tauDesFunc;
};

#endif