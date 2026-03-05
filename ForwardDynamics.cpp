#include <OpenSim/OpenSim.h>
#include "SeriesElasticActuator.h"
#include "SEATrackingController.h"
#include "InitModel.h"
#include <OpenSim/Analyses/ForceReporter.h>
#include <iostream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main() {
    for(int iter = 0; iter < 1; iter++){
        try {
            cout << "--- Avvio Simulazione N." << iter + 1 << " SEA Tracking ---" << endl;

            Object::registerType(SeriesElasticActuator());
            // Real model with SEA
            Model model("C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA-plugin-OpenSim - core\\Adjusted_sea.osim");
            
            double Kp[10] = {1000, 1000, 1000, 100, 100, 100, 5000, 5000, 10000, 10};            
            double Kd[10] = { 20,    5,  250,  30,  10, 100,  220,   50,   315, 50};
            //double Kd = 2*sqrt(Kp);
            
            /* First argument: Actuation_force.sto file of the "ideal model" (CMC output)
            Second argument: Name of the actuator in the CMC file (e.g., "reserve_pros_knee_angle")
            Third argument: Name of your SEA in the real model (e.g., "SEA") */

            std::string TauRef_file = "C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA-plugin-OpenSim - core\\3DGaitModel2392_Actuation_force.sto";

            auto controller = new SEATrackingController(TauRef_file, "reserve_pros_knee_angle", "SEA", Kp[iter], Kd[iter]);
            cout<< "Controller creato con successo!" << endl;
            controller->addActuator(model.getComponent<SeriesElasticActuator>("forceset/SEA"));
            model.addController(controller);

            ForceReporter* forceReporter = new ForceReporter(&model);
            model.addAnalysis(forceReporter);

            model.setUseVisualizer(false); 
            State& s = model.initSystem();
            
            std::string Kinematic_refs_file = "C:\\Users\\tomma\\Desktop\\Opensim OMNIBUS\\SEA_plugin_core\\3DGaitModel2392_Kinematics_q.sto";
            Storage KinRefFile(Kinematic_refs_file);

            double startTime = 4.26;
            s.setTime(startTime);


            initializeModel(model, s, KinRefFile, startTime);
            
            model.assemble(s);
            Manager manager(model);
            manager.getIntegrator().setAccuracy(1.0e-5);
            manager.getIntegrator().setMinimumStepSize(1e-8);
            
            manager.initialize(s);
            cout << "Simulazione in corso..." << endl;
            manager.integrate(11.07); 

            // 4. Salva i risultati
            /* manager.getStateStorage().print("C:\\Users\\tomma\\Desktop\\Results\\SEA_Tracking_Results_States.sto");
            auto controlsTable = model.getControlsTable();
            STOFileAdapter::write(controlsTable, "C:\\Users\\tomma\\Desktop\\Results\\SEA_Tracking_Results_Controls.sto");
            */         
            
            forceReporter->getForceStorage().print("C:\\Users\\tomma\\Desktop\\Results\\" + std::to_string(iter + 1) + " - SEA_Kp=" + std::to_string((int)Kp[iter]) + "_Kd=" + std::to_string((int)Kd[iter]) + ".sto");
            cout << "Simulazione completata con successo!" << endl;

        } catch (const OpenSim::Exception& ex) {
            cout << "Errore in OpenSim: " << ex.getMessage() << endl;
            return 1;
        } catch (const std::exception& ex) {
            cout << "Errore C++: " << ex.what() << endl;
            return 1;
        }
    }
    return 0;

}

