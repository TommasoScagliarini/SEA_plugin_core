#ifndef INITMODEL_H
#define INITMODEL_H

#include <OpenSim/OpenSim.h>
#include "SeriesElasticActuator.h"
#include <iostream>


using namespace OpenSim;
using namespace SimTK;
using namespace std;

void initializeModel(Model& model, State& s, Storage KinRefFile, double startTime);

void initializeModel(Model& model, State& s, Storage KinRefFile, double startTime) {
    const CoordinateSet& coordSet = model.getCoordinateSet();
    Array<double> stateValues;
    stateValues.setSize(coordSet.getSize());
    KinRefFile.getDataAtTime(startTime, coordSet.getSize(), stateValues);

    Array<std::string> colNames = KinRefFile.getColumnLabels();

    for (int col = 1; col < colNames.getSize(); ++col) { // col=0 è "time"
            std::string coordName = colNames[col];
            
            // Rimuovi eventuale suffisso "/value" se presente
            auto slashPos = coordName.rfind('/');
            if (slashPos != std::string::npos) {
                coordName = coordName.substr(slashPos + 1);
            }
            
            // Controlla che la coordinata esista nel modello
            if (!model.getCoordinateSet().contains(coordName)) {
                cout << "[WARN] Coordinata '" << coordName << "' nel .sto non trovata nel modello. Skippata." << endl;
                continue;
            }
            
            // ATTENZIONE: Rimosso 'const' per poter usare setLocked
            const Coordinate& coord = model.getCoordinateSet().get(coordName);
            double val = stateValues[col - 1]; // stateValues non include la colonna time
            
            // Conversione gradi -> radianti per coordinate rotazionali
            if (coord.getMotionType() == Coordinate::Rotational) {
                val = SimTK::convertDegreesToRadians(val);
            }
            
            // 1. Imposta il valore nel sistema
            coord.setValue(s, val);

            // 2. BLOCCO DELLE COORDINATE: Blocca tutto tranne il ginocchio con il SEA
            //if (coordName != "pros_knee_angle") {
                coord.setLocked(s, true);
            //}
            
            // 3. Stampa a schermo indicando se è stata bloccata
            cout << "[INIT] " << coordName << " = " << val << " rad" 
                 << (coord.getLocked(s) ? " (LOCKED)" : "") << endl;
        }

        auto& sea = model.getComponent<SeriesElasticActuator>("forceset/SEA");
        double knee_angle = model.getCoordinateSet().get("pros_knee_angle").getValue(s);
        sea.setStateVariableValue(s, "motor_angle", knee_angle);
        sea.setStateVariableValue(s, "motor_speed", 0.0);
    
}

#endif