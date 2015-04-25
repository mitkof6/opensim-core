/* -------------------------------------------------------------------------- *
*                         OpenSim:  testKneeLigament.cpp                     *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2015 Stanford University and the Authors                *
* Author(s): Dimitar Stanev                                                  *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */

//============================================================================
//
//============================================================================

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Ligament.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Actuators/SpringGeneralizedForce.h>

using std::cout;
using std::endl;
using std::string;
using namespace OpenSim;
using namespace SimTK;


void simulate(Model& model, State& state);

void constructModel(Model& model);

int main()
{
	try {
		Model model;

		constructModel(model);

		model.print("block_ligament.osim");

		ForceReporter* force_analysis = new ForceReporter(&model);
		model.addAnalysis(force_analysis);

		SystemEnergyProbe* energy = new SystemEnergyProbe(true, true);
		energy->setName("total_energy");
		model.addProbe(energy);

		ProbeReporter* probe_reporter = new ProbeReporter(&model);
		model.addAnalysis(probe_reporter);

		model.buildSystem();

		State& s = model.initializeState();

		simulate(model, s);

		force_analysis->printResults("ligament");
		probe_reporter->printResults("ligament");


	}
	catch (const std::exception& ex)
	{
		cout << "Exception: " << ex.what() << endl;
	}
	catch (...)
	{
		cout << "Unrecognized exception " << endl;
	}
	system("pause");
	return 0;
}

void constructModel(Model& model)
{
	OpenSim::Ground ground = model.getGround();

	double blockMass = 20.0, blockSideLength = 0.1; Vec3 blockMassCenter(0);
	Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

	OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);
	block->addDisplayGeometry("block.vtp");

	Vec3 locationInParent(0, blockSideLength / 2, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
	FreeJoint* blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInParent, orientationInBody);
	model.addJoint(blockToGround);

	model.addBody(block);

	State dummy = State();

	//create ligament
	Ligament* ligament = new Ligament();
	ligament->setName("ligament");

	//create point path
	PathPointSet* ligament_path = new PathPointSet();

	PathPoint ground_point = PathPoint();
	ground_point.setName("ligament_ground");
	ground_point.setBody(ground);
	ground_point.setLocation(dummy, Vec3(0));
	ligament_path->insert(0, ground_point);

	PathPoint block_point = PathPoint();
	block_point.setName("ligament_block");
	block_point.setBody(model.updBodySet().get("block"));
	block_point.setLocation(dummy, Vec3(0));
	ligament_path->insert(1, block_point);

	//create geometry path
	GeometryPath* geometry_path = new GeometryPath();
	geometry_path->setName("ligament_geometry_path");
	geometry_path->updPathPointSet() = *ligament_path;
	ligament->updGeometryPath() = *geometry_path;

	//parameters
	ligament->setRestingLength(0.5);
	ligament->setEpsilonLength(0.01);
	ligament->setLigamentStiffnessg(1000);

	model.addForce(ligament);

	//SpringGeneralizedForce* spring = new SpringGeneralizedForce("blockToGround_yTranslation");
	//spring->setStiffness(1000);
	//spring->setRestLength(-0.5);
	//model.addForce(spring);

	model.setUseVisualizer(true);

}


void simulate(Model& model, State& state)
{
	//setup integrator
	RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(1E-7);
	integrator.setMaximumStepSize(1E-3);

	//manager
	Manager manager(model, integrator);
	manager.setInitialTime(0);
	manager.setFinalTime(3);

	//integrate
	clock_t begin = clock();
	cout << "Integrating ..." << endl;
	manager.integrate(state);
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Elapsed time: " << elapsed_secs << endl;
	cout.flush();

	//state results
	OpenSim::Storage state_reporter(manager.getStateStorage());
	model.updSimbodyEngine().convertRadiansToDegrees(state_reporter);
	state_reporter.print("ligament.sto");
}

