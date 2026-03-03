#include <OpenSim/OpenSim.h>
#include "SeriesElasticActuator.h"
#include "SEATrackingController.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main() {
    try {
        cout << "--- Avvio Simulazione SEA Tracking ---" << endl;

        Object::registerType(SeriesElasticActuator());
        // Real model with SEA
        Model model("C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA-plugin-OpenSim - core\\Adjusted_sea.osim");
        
        double Kp = 100; 
        double Kd = 2.5; 
        
        /* First argument: Actuation_force.sto file of the "ideal model" (CMC output)
        Second argument: Name of the actuator in the CMC file (e.g., "reserve_pros_knee_angle")
        Third argument: Name of your SEA in the real model (e.g., "SEA") */

        std::string TauRef_file = "C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA-plugin-OpenSim - core\\3DGaitModel2392_Actuation_force.sto";

        auto controller = new SEATrackingController(TauRef_file, "reserve_pros_knee_angle", "SEA", Kp, Kd);
        cout<< "Controller creato con successo!" << endl;
        controller->addActuator(model.getComponent<SeriesElasticActuator>("forceset/SEA"));
        model.addController(controller);

        model.setUseVisualizer(true); 
        State& s = model.initSystem();
        
        std::string Kinematic_refs_file = "C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA_plugin_core\\3DGaitModel2392_Kinematics_q.sto";
        Storage KinRefFile(Kinematic_refs_file);

        double startTime = 4.26;
        s.setTime(startTime);

        const CoordinateSet& coordSet = model.getCoordinateSet();
        Array<double> stateValues;
        stateValues.setSize(coordSet.getSize());
        KinRefFile.getDataAtTime(startTime, coordSet.getSize(), stateValues);

        Array<std::string> colNames = KinRefFile.getColumnLabels();

        /* for (int col = 1; col < colNames.getSize(); ++col) { // col=0 è "time"
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
            
            const Coordinate& coord = model.getCoordinateSet().get(coordName);
            double val = stateValues[col - 1]; // stateValues non include la colonna time
            
            // Conversione gradi -> radianti per coordinate rotazionali
            if (coord.getMotionType() == Coordinate::Rotational) {
                val = SimTK::convertDegreesToRadians(val);
            }
            
            coord.setValue(s, val);
            cout << "[INIT] " << coordName << " = " << val << " rad" << endl;
        } */
        
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
            if (coordName != "pros_knee_angle") {
                coord.setLocked(s, true);
            }
            
            // 3. Stampa a schermo indicando se è stata bloccata
            cout << "[INIT] " << coordName << " = " << val << " rad" 
                 << (coord.getLocked(s) ? " (LOCKED)" : "") << endl;
        }

        auto& sea = model.getComponent<SeriesElasticActuator>("forceset/SEA");
        double knee_angle = model.getCoordinateSet().get("pros_knee_angle").getValue(s);
        sea.setStateVariableValue(s, "motor_angle", knee_angle);
        sea.setStateVariableValue(s, "motor_speed", 0.0);
        
        model.assemble(s);
        Manager manager(model);
        manager.getIntegrator().setAccuracy(1.0e-5);
        manager.getIntegrator().setMinimumStepSize(1e-8);
        
        manager.initialize(s);
        cout << "Simulazione in corso..." << endl;
        manager.integrate(11.07); 

        // 4. Salva i risultati
        manager.getStateStorage().print("C:\\Users\\tomma\\Desktop\\ResultsSEA_Tracking_Results_States.sto");
        auto controlsTable = model.getControlsTable();
        STOFileAdapter::write(controlsTable, "C:\\Users\\tomma\\Desktop\\ResultsSEA_Tracking_Results_Controls.sto");

        cout << "Simulazione completata con successo!" << endl;

    } catch (const OpenSim::Exception& ex) {
        cout << "Errore in OpenSim: " << ex.getMessage() << endl;
        return 1;
    } catch (const std::exception& ex) {
        cout << "Errore C++: " << ex.what() << endl;
        return 1;
    }
    return 0;
}

