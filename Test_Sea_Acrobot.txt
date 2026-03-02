#include <OpenSim/OpenSim.h>
#include "SeriesElasticActuator.h" 
#include "CustomControl.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main() {
    try {
        cout << "--- Creazione Modello Acrobot ---" << endl;

        // 1. Definizione del Modello
        Model model;
        model.setName("Acrobot_SEA");
        model.setGravity(Vec3(0, -9.81, 0)); // g = 9.81 
        //Ground& ground = model.updGround();

        // --- Parametri (Set 1 della Tabella 1)  ---
        double m1 = 1.0;
        double m2 = 1.0;
        double l1 = 1.0;
        double l2 = 1.0;
        double lc1 = 0.5; // Distanza pivot-COM link 1
        double lc2 = 0.5; // Distanza pivot-COM link 2
        double I1 = 0.33; // Inerzia link 1
        double I2 = 0.33; // Inerzia link 2

        // 2. Corpo 1 (Link 1)
        // Il body è definito rispetto al suo centro di massa (COM).
        // Poiché il link è lungo l1 e il COM è a lc1 dal pivot, 
        // la geometria visiva sarà offsettata.
        OpenSim::Body* link1 = new OpenSim::Body("link1", m1, Vec3(0), Inertia(I1));
        
        // Geometria Link 1 (Cilindro rosso)
        Geometry* geom1 = new Cylinder(0.05, l1 / 2.0);
        geom1->setColor(Vec3(0.8, 0.1, 0.1));
        
        // Posizioniamo la geometria in modo che il cilindro copra la lunghezza l1
        // Il COM è a lc1 dal pivot. 
        Frame* geomFrame1 = new PhysicalOffsetFrame(*link1, Transform(Vec3(0, 0, 0))); 
        geomFrame1->attachGeometry(geom1);
        model.addComponent(geomFrame1);
        model.addBody(link1);

        // 3. Corpo 2 (Link 2)
        OpenSim::Body* link2 = new OpenSim::Body("link2", m2, Vec3(0), Inertia(I2));
        
        // Geometria Link 2 (Cilindro blu)
        Geometry* geom2 = new Cylinder(0.05, l2 / 2.0);
        geom2->setColor(Vec3(0.1, 0.1, 0.8));
        Frame* geomFrame2 = new PhysicalOffsetFrame(*link2, Transform(Vec3(0, 0, 0)));
        geomFrame2->attachGeometry(geom2);
        model.addComponent(geomFrame2);
        model.addBody(link2);

        Sphere* sphereTop = new Sphere(0.06); // Raggio leggermente più grande del cilindro
        //sphereTop->setColor(Vec3(0.1, 0.8, 0.1)); // Verde
        Frame* sphereFrame1 = new PhysicalOffsetFrame(*link2, Transform(Vec3(0, lc2, 0)));
        sphereFrame1->attachGeometry(sphereTop);
        model.addComponent(sphereFrame1);

        // 2. Sfera all'estremità inferiore (Tip)
        // Posizione: (lc2 - l2) sotto il COM
        // Spiegazione: Dal pivot scendiamo di tutta la lunghezza l2. 
        // Poiché il pivot è a +lc2, la punta è a (lc2 - l2).
        Sphere* sphereBot = new Sphere(0.06);
        //sphereBot->setColor(Vec3(0.8, 0.1, 0.8)); // Viola
        Frame* sphereFrame2 = new PhysicalOffsetFrame(*link2, Transform(Vec3(0, lc2 - l2, 0)));
        sphereFrame2->attachGeometry(sphereBot);
        model.addComponent(sphereFrame2);

        // 4. Giunto 1 (Spalla/Hip) - Tra Ground e Link 1
        // Ancorato all'origine del Ground (0,0,0)
        // Collegato al Link 1 a distanza lc1 sopra il COM (perché il COM è 'sotto' il pivot)
        Vec3 locInParent1(0, 2.5, 0);
        Vec3 locInChild1(0, lc1, 0); 
        
        PinJoint* joint1 = new PinJoint("theta1", model.getGround(), locInParent1, Vec3(0), *link1, locInChild1, Vec3(0));
        
        // Impostiamo le coordinate (theta1)
        Coordinate& theta1 = joint1->updCoordinate();
        theta1.setName("theta1");
        theta1.setDefaultValue(0); // Posizione verticale giù
        model.addJoint(joint1);

        // 5. Giunto 2 (Gomito) - Tra Link 1 e Link 2
        // Collegato alla fine del Link 1 (che è lc1 + (l1-lc1) = l1 sotto il pivot, quindi lc1 - l1 sotto il COM?)
        // Facciamo i conti: Pivot1 è a +lc1 dal COM1. La fine del link è a - (l1 - lc1) dal COM1.
        // Se l1=1 e lc1=0.5, pivot è a +0.5, fine è a -0.5.
        Vec3 locInParent2(0, -(l1 - lc1), 0); 
        Vec3 locInChild2(0, lc2, 0); // Pivot del link 2 è sopra il suo COM
        
        PinJoint* joint2 = new PinJoint("theta2", *link1, locInParent2, Vec3(0), *link2, locInChild2, Vec3(0));
        
        Coordinate& theta2 = joint2->updCoordinate();
        theta2.setName("theta2");
        theta2.setDefaultValue(0); 
        model.addJoint(joint2);

        // 6. Attuatore (Torque) sul secondo giunto
        // L'assignment specifica: "Input is the torque on the second link" [cite: 12]
        SeriesElasticActuator* sea = new SeriesElasticActuator("SEA", 0.1, 1, 100.0);
        sea->connectSocket_coordinate(joint2->getCoordinate());        
        //sea->setOptimalForce(1.0); // Scala della forza
        model.addForce(sea);


        bool CustomControl = true;
        // 7. Controller di esempio (Sinusoide per vedere il movimento)
        if (CustomControl){
            CustomController* controller = new CustomController(50.0, 2.5); // kp=10, kv=5 3.5
            controller->setActuators(model.updActuators());
            controller->connectSocket_coordinate(joint2->getCoordinate());
            model.addController(controller);
        }else{
            PrescribedController* controller = new PrescribedController();
            controller->addActuator(*sea);
            // Applica una coppia sinusoidale: 5 * sin(t)
            controller->prescribeControlForActuator("SEA", new Sine(5.0, 1.0, 0.0));
            //controller->prescribeControlForActuator("SEA", new Constant(0)); 
            model.addController(controller);
        }
        
        // 8. Inizializzazione e Simulazione
        model.setUseVisualizer(true);
        State& s = model.initSystem();
        
        cout << "=== Modello inizializzato ===" << endl;
        
        Manager manager(model);
        manager.initialize(s);
        manager.integrate(10.0); // Simula per 10 secondi

        // 9. Output
        manager.getStateStorage().print("/Users/tommy/Documents/Intership_OpenSim/SEA-plugin-OpenSim/build/Acrobot_Results.sto");
        
        auto controlsTable = model.getControlsTable();
        STOFileAdapter::write(controlsTable, "/Users/tommy/Documents/Intership_OpenSim/SEA-plugin-OpenSim/build/Acrobat_controls.sto");
        
        // auto forcesTable = forces->getForcesTable();
        // STOFileAdapter::write(forcesTable, "actuator_forces.sto");
        
        model.print("/Users/tommy/Documents/Intership_OpenSim/SEA-plugin-OpenSim/build/Acrobot.osim");
        cout << "--- Simulazione completata. Generato Acrobot.osim ---" << endl;

    } catch (const std::exception& ex) {
        cout << "ERRORE: " << ex.what() << endl;
        return 1;
    }
    return 0;
}