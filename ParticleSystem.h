//#pragma once
#ifndef PARTICLE_SYS_H
#define PARTICLE_SYS_H

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>

#include "shared/opengl.h"

#include "Particle.h"
#include "Spring.h"		

class ParticleSystem : public BaseSystem
{

public:
	ParticleSystem(const std::string& name);
	virtual void getState(Particle* p);
	virtual void setState(Particle* p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { pModel.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&pModel); }
	int command(int argc, myCONST_SPEC char** argv);

	Particle* getParticles();
	Particle& getParticle(int i);
	int getNumParticles();

	Spring* getSprings();
	Spring& getCurrSpr();
	Spring& getSpring(int i);
	int getNumSprings();
	int getSprInUse();
	void setNumSprings(int numSpr);
	void setSprInUse(int numSpr);
	void setNumParticles(int numPart);

protected:
	GLMmodel pModel;
	
	Particle particles[1001];
	Spring springs[1001];
	
	int numParticles, numSprings, sprInUse;

	bool spheresSelected;
	bool pointsSelected;

};
#endif
