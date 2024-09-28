//#pragma once
#ifndef PARTICLE_H
#define PARTICLE_H

#include <util/util.h>

class Particle {

private:
	Vector pos;
	Vector firstPos;
	Vector prevPos;
	Vector vel;
	Vector firstVel;
	Vector acc;
	Vector force;
	double mass;
	bool isFixed;

public:
	Particle();
	Particle(Vector pos, Vector firstPos, Vector prevPos, Vector vel, Vector firstVel, Vector force, double mass, bool isFixed);

	Vector& getPos();
	Vector& getFirstPos();
	Vector& getPrevPos();
	Vector& getVel();
	Vector& getFirstVel();
	Vector& getAcc();
	Vector& getForce();
	double getMass();
	bool getIsFixed();

	void setPos(double x, double y, double z);
	void setFirstPos(double x, double y, double z);
	void setPrevPos(double x, double y, double z);
	void setVel(double x, double y, double z);
	void setFirstVel(double x, double y, double z);
	void setAcc(double x, double y, double z);
	void setForce(double x, double y, double z);
	void setMass(double mass);
	void setIsFixed(bool isFixed);
	void nailParticle();

};

#endif
