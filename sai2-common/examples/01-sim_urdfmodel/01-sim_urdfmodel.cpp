// This example application loads a simple robot from a URDF file and simulates
// its physics in a Dynamics3D virtual world.

#include "simulation/SimulationInterface.h"

#include <iostream>
#include <string>
#include <vector>

using namespace std;

const string world_file = "resources/world.urdf";
const vector<string> robot_names = {"PBot1", "PBot2"};

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	auto sim = new Simulation::SimulationInterface(world_file, Simulation::sai2simulation, Simulation::urdf, true);

	// integrate
	Eigen::VectorXd q(sim->dof(robot_names[0]));
	for (;;) {
		sim->integrate(0.0005);
		sim->getJointPositions(robot_names[0], q);
		cout << "Joint 0: " << q[0] << endl;
	}

	return 0;
}
