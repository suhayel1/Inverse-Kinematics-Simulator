//#pragma once
#ifndef SPRING_H
#define SPRING_H

#include <util/util.h>
#include "Particle.h"

class Spring {

private:
	int i, j;
	double ks, kd, rl;

public:
	Spring();
	Spring(int i, int j, double ks, double kd, double rl);

	int getI();
	int getJ();
	double getKS();
	double getKD();
	double getRL();

	void setI(int i);
	void setJ(int j);
	void setKS(double ks);
	void setKD(double kd);
	void setRL(double rl);

	void springForce(Particle& i, Particle& j);
	void damperForce(Particle& i, Particle& j);
};

#endif
