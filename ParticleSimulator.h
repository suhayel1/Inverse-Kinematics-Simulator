//#pragma once
#ifndef PARTICLE_SIM_H
#define PARTICLE_SIM_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"

#include <string>

#include "ParticleSystem.h"
#include "GlobalResourceManager.h"

class ParticleSimulator : public BaseSimulator
{
public:

	ParticleSimulator(const std::string& name, BaseSystem* target);
	~ParticleSimulator();

	int step(double time);
	int init(double time)
	{
		//m_object->getState(m_pos0);
		//setVector(m_vel0, 0, 0, 0);
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

	void totalForce(int i);
	void normForce(int i, double detect);
	void eulerVelocity(int i);
	void eulerPosition(int i);
	void verlet(int i);

	void setLmbHeldDown(bool lmbHeldDown);
	bool getLmbHeldDown();

	void setMousePos(double x, double y, double z);
	Vector& getMousePos();

	void pickFromXYPlane(Vector result, int x, int y);

	ParticleSystem* getObject();

	static double dt;

protected:
	ParticleSystem* pObject;

	Vector grav;
	Vector norm;
	Vector point;
	Vector mousePos;

	std::string sysName, integMethod;

	double grKS, grKD, kDrag;

	bool lmbIsHeldDown;

	Particle* objPart1;
	Particle* objPart2;

	Spring* objSpr;

	Vector diff;

	double distFromPtr, minDistFromPtr;

	int mouseIndex;
};

#endif