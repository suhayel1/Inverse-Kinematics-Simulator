#include "Particle.h"

Particle::Particle() {
	zeroVector(this->pos);
	zeroVector(this->firstPos);
	zeroVector(this->prevPos);
	zeroVector(this->vel);
	zeroVector(this->firstVel);
	zeroVector(this->acc);
	zeroVector(this->force);
	this->mass = 1;
	this->isFixed = false;
}

Particle::Particle(Vector pos, Vector firstPos, Vector prevPos, Vector vel, Vector firstVel, Vector force, double mass, bool isFixed) {
	VecCopy(this->pos, pos);
	VecCopy(this->firstPos, firstPos);
	VecCopy(this->prevPos, prevPos);
	VecCopy(this->vel, vel);
	VecCopy(this->firstVel, firstVel);
	VecCopy(this->force, force);
	this->mass = mass;
	this->isFixed = isFixed;

	VecCopy(this->acc, this->force);
	VecScale(this->acc, 1.0 / this->mass);
}

Vector& Particle::getPos() {
	return this->pos;
}

Vector& Particle::getFirstPos() {
	return this->firstPos;
}

Vector& Particle::getPrevPos() {
	return this->prevPos;
}

Vector& Particle::getVel() {
	return this->vel;
}

Vector& Particle::getFirstVel() {
	return this->firstVel;
}

Vector& Particle::getAcc() {
	return this->acc;
}

Vector& Particle::getForce() {
	return this->force;
}

double Particle::getMass() {
	return this->mass;
}

bool Particle::getIsFixed() {
	return this->isFixed;
}

void Particle::setPos(double x, double y, double z) {
	setVector(this->pos, x, y, z);
}

void Particle::setFirstPos(double x, double y, double z) {
	setVector(this->firstPos, x, y, z);
}

void Particle::setPrevPos(double x, double y, double z) {
	setVector(this->prevPos, x, y, z);
}

void Particle::setVel(double x, double y, double z) {
	setVector(this->vel, x, y, z);
}

void Particle::setFirstVel(double x, double y, double z) {
	setVector(this->firstVel, x, y, z);
}

void Particle::setAcc(double x, double y, double z) {
	setVector(this->acc, x, y, z);
}

void Particle::setForce(double x, double y, double z) {
	setVector(this->force, x, y, z);
}

void Particle::setMass(double mass) {
	this->mass = mass;
}

void Particle::setIsFixed(bool isFixed) {
	this->isFixed = isFixed;
}

void Particle::nailParticle() {
	this->isFixed = true;
}
