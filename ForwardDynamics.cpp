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
        
        double Kp = 1; 
        double Kd = 0; 
        
        /* First argument: Actuation_force.sto file of the "ideal model" (CMC output)
        Second argument: Name of the actuator in the CMC file (e.g., "reserve_pros_knee_angle")
        Third argument: Name of your SEA in the real model (e.g., "SEA") */

        std::string TauRef_file = "C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA-plugin-OpenSim - core\\3DGaitModel2392_Actuation_force.sto";

        auto controller = new SEATrackingController(TauRef_file, "reserve_pros_knee_angle", "SEA", Kp, Kd);
        cout<< "Controller creato con successo!" << endl;
        controller->addActuator(model.getComponent<SeriesElasticActuator>("forceset/SEA"));
        model.addController(controller);

        // model.setUseVisualizer(true); 
        State& s = model.initSystem();
        
        std::string Kinematic_refs_file = "C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA_plugin_core\\3DGaitModel2392_Kinematics_q.sto";
        Storage KinRefFile(Kinematic_refs_file);

        double startTime = 4.26;
        s.setTime(startTime);

        const CoordinateSet& coordSet = model.getCoordinateSet();
        Array<double> stateValues;
        stateValues.setSize(coordSet.getSize());
        KinRefFile.getDataAtTime(startTime, coordSet.getSize(), stateValues);

        for (int i = 0; i < stateValues.getSize(); ++i) {
            coordSet.get(i).setValue(s, stateValues[i]);
            //cout << "[" << i << "] "<< "coordinate set " << coordSet.get(i).getName() << ": " << stateValues[i] << endl;
        }
        
        model.assemble(s);
        Manager manager(model);
        manager.getIntegrator().setAccuracy(1.0e-3);
        
        manager.initialize(s);
        cout << "Simulazione in corso..." << endl;
        manager.integrate(5.0); 

        // 4. Salva i risultati
        manager.getStateStorage().print("SEA_Tracking_Results_States.sto");
        auto controlsTable = model.getControlsTable();
        STOFileAdapter::write(controlsTable, "SEA_Tracking_Results_Controls.sto");

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