#include "Spring.h"

Spring::Spring() {
	this->i = -1;
	this->j = -1;
	this->ks = -1;
	this->kd = -1;
	this->rl = -1;
}

Spring::Spring(int i, int j, double ks, double kd, double rl) {
	this->i = i;
	this->j = j;
	this->ks = ks;
	this->kd = kd;
	this->rl = rl;
}


int Spring::getI() {
	return this->i;
}

int Spring::getJ() {
	return this->j;
}


double Spring::getKS() {
	return this->ks;
}

double Spring::getKD() {
	return this->kd;
}

double Spring::getRL() {
	return this->rl;
}


void Spring::setI(int i) {
	this->i = i;
}

void Spring::setJ(int j) {
	this->j = j;
}


void Spring::setKS(double ks) {
	this->ks = ks;
}

void Spring::setKD(double kd) {
	this->kd = kd;
}

void Spring::setRL(double rl) {
	this->rl = rl;
}

void Spring::springForce(Particle& i, Particle& j) {

	Vector temp;
	Vector ijLenVec;
	double ijLen;

	VecSubtract(ijLenVec, i.getPos(), j.getPos());
	ijLen = VecLength(ijLenVec);

	VecCopy(temp, ijLenVec);
	VecScale(temp, this->ks * (this->rl - ijLen) * (1.0 / ijLen));
	VecAdd(i.getForce(), i.getForce(), temp);

	VecScale(temp, -1);
	VecAdd(j.getForce(), j.getForce(), temp);

}

void Spring::damperForce(Particle& i, Particle& j) {

	Vector temp;
	Vector ijLenVec;
	Vector ijVel;
	double ijLen;

	VecSubtract(ijLenVec, i.getPos(), j.getPos());
	VecSubtract(ijVel, i.getVel(), j.getVel());
	ijLen = VecLength(ijLenVec);
	VecScale(ijLenVec, (1.0 / ijLen));

	VecCopy(temp, ijLenVec);
	VecScale(temp, -this->kd * VecDotProd(ijVel, ijLenVec));
	VecAdd(i.getForce(), i.getForce(), temp);

	VecScale(temp, -1);
	VecAdd(j.getForce(), j.getForce(), temp);
}
