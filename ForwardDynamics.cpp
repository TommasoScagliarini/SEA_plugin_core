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

        // model.setUseVisualizer(true); 
        State& s = model.initSystem();
        
        // Se necessario, imposta lo stato iniziale (theta_q e velocità) copiandolo dal primo frame della cinematica
        // ...
        
        Manager manager(model);
        manager.initialize(s);
        cout << "Simulazione in corso..." << endl;
        manager.integrate(1.0); 

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