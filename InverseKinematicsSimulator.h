#pragma once
#ifndef INVERSE_KINEMATICS_SIM_H
#define INVERSE_KINEMATICS_SIM_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"

#include <string>

#include "HumanSystem.h"
#include "Hermite.h"
#include <util/myMath.h>
#include <util/Jama/jama_lu.h>

// a sample simulator

class InverseKinematicsSimulator : public BaseSimulator
{
public:

	InverseKinematicsSimulator(const std::string& name, BaseSystem* target1, BaseSystem* target2);
	~InverseKinematicsSimulator();

	int step(double time);
	int init(double time);

	int command(int argc, myCONST_SPEC char** argv);

protected:

	HumanSystem* human;
	Hermite* herm;

	double currentP[4], targetP[4], pTargetP[4];
	double deltaX[3], deltaTheta[7];
	double jacobian[3][7];

	Vector err, threshold;

	VectorObj p;

	double u;
	double du = 0.0001;

	void IKSolve();
	void IKSolver();
	void calcJ();
	void endEffInWorldCS();

};


#endif
