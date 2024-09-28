#include "ParticleSystem.h"

ParticleSystem::ParticleSystem(const std::string& name) :
	BaseSystem(name),
	numParticles(0),
	numSprings(0),
	sprInUse(0)
{
	pointsSelected = true;
	spheresSelected = false;
}	// ParticleSystem

void ParticleSystem::getState(Particle* p)
{
	p = particles;
}	// ParticleSystem::getState

void ParticleSystem::setState(Particle* p)
{
	for (int i = 0; i < numParticles; i++) {
		particles[i] = *(p + i);
	}
}	// ParticleSystem::setState

void ParticleSystem::reset(double time)
{
	for (int i = 0; i < numParticles; i++) {
		VecCopy(particles[i].getPos(), particles[i].getFirstPos());
		VecCopy(particles[i].getPrevPos(), particles[i].getFirstPos());
		VecCopy(particles[i].getVel(), particles[i].getFirstVel());
		zeroVector(particles[i].getAcc());
		zeroVector(particles[i].getForce());
		particles[i].setIsFixed(false);
	}
}	// ParticleSystem::Reset


int ParticleSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "dim") == 0)
	{
		if (argc == 2)
		{
			numParticles = atoi(argv[1]);

			if (numParticles < 1 || numParticles > 1000) {
				animTcl::OutputMessage("Invalid number of particles");
				return TCL_ERROR;
			}

			for (int i = 0; i < numParticles; i++) {
				particles[i].setPos(0, 0, 0);
				particles[i].setFirstPos(0, 0, 0);
				particles[i].setPrevPos(0, 0, 0);
				particles[i].setVel(0, 0, 0);
				particles[i].setFirstVel(0, 0, 0);
				particles[i].setAcc(0, 0, 0);
				particles[i].setForce(0, 0, 0);
				particles[i].setMass(1);
				particles[i].setIsFixed(false);
			}
			//return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: dim <Number of Particles> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "particle") == 0)
	{
		if (argc == 9)
		{
			int i = atoi(argv[1]);

			if (i < numParticles) {
				particles[i].setMass(atof(argv[2]));
				particles[i].setPos(atof(argv[3]), atof(argv[4]), atof(argv[5]));
				particles[i].setFirstPos(atof(argv[3]), atof(argv[4]), atof(argv[5]));
				particles[i].setPrevPos(atof(argv[3]), atof(argv[4]), atof(argv[5]));
				particles[i].setVel(atof(argv[6]), atof(argv[7]), atof(argv[8]));
				particles[i].setFirstVel(atof(argv[6]), atof(argv[7]), atof(argv[8]));
			}
			else {
				animTcl::OutputMessage("Index out of bounds");
				return TCL_ERROR;
			}
			//return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: particle <index> <mass> <x y z vx vy vz> ");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "all_velocities") == 0)
	{
		if (argc == 4)
		{
			for (int i = 0; i < numParticles; i++) {
				particles[i].setVel(atof(argv[1]), atof(argv[2]), atof(argv[3]));
			}
			//return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: all_velocities <vx vy vz> ");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "use_spheres") == 0)
	{
		if (argc != 1)
		{
			animTcl::OutputMessage("Usage: use_spheres");
			return TCL_ERROR;
		}
		else {
			spheresSelected = true;
			pointsSelected = false;
		}
	}
	else if (strcmp(argv[0], "use_points") == 0)
	{
		if (argc != 1)
		{
			animTcl::OutputMessage("Usage: use_points");
			return TCL_ERROR;
		}
		else {
			spheresSelected = false;
			pointsSelected = true;
		}
	}
	else {
		animTcl::OutputMessage("Command not found");
		return TCL_ERROR;
	}

	glutPostRedisplay();
	return TCL_OK;

}	// ParticleSystem::command

void ParticleSystem::display(GLenum mode)
{
	Vector pos, pos1, pos2;

	if (pointsSelected) {
		glPointSize(4.0);
		glDisable(GL_LIGHTING);
		glDisable(GL_COLOR_MATERIAL);

		glBegin(GL_POINTS);
	}

	if (spheresSelected) {
		glEnable(GL_LIGHTING);
		glEnable(GL_COLOR_MATERIAL);
		glMatrixMode(GL_MODELVIEW);
	}

	glColor3f(0.5f, 0.0f, 0.0f);	// dark red

	for (int i = 0; i < numParticles; i++) {
		VecCopy(pos, particles[i].getPos());

		if (spheresSelected) {
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glTranslated(pos[0], pos[1], pos[2]);
			glScalef(0.1, 0.1, 0.1);
			glutSolidSphere(1.0, 20, 20);
			glPopMatrix();
			glPopAttrib();
		}

		if (pointsSelected) glVertex3f(pos[0], pos[1], pos[2]);

	}

	if (pointsSelected) glEnd();

	if (spheresSelected) {
		glDisable(GL_LIGHTING);
		glDisable(GL_COLOR_MATERIAL);
	}

	glColor3f(1.0f, 1.0f, 1.0f);	// white

	glBegin(GL_LINES);

	for (int j = 0; j < sprInUse; j++) {
		VecCopy(pos1, particles[springs[j].getI()].getPos());
		VecCopy(pos2, particles[springs[j].getJ()].getPos());
		glVertex3f(pos1[0], pos1[1], pos1[2]);
		glVertex3f(pos2[0], pos2[1], pos2[2]);
	}

	glEnd();

}	// ParticleSystem::display

Particle* ParticleSystem::getParticles() {
	return this->particles;
}

Particle& ParticleSystem::getParticle(int i) {
	return this->particles[i];
}

int ParticleSystem::getNumParticles() {
	return this->numParticles;
}

Spring* ParticleSystem::getSprings() {
	return this->springs;
}

int ParticleSystem::getNumSprings() {
	return this->numSprings;
}

int ParticleSystem::getSprInUse() {
	return this->sprInUse;
}

void ParticleSystem::setNumSprings(int numSpr) {
	this->numSprings = numSpr;
}

void ParticleSystem::setSprInUse(int numSpr) {
	this->sprInUse = numSpr;
}

Spring& ParticleSystem::getCurrSpr() {
	return this->springs[sprInUse];
}

Spring& ParticleSystem::getSpring(int i) {
	return this->springs[i];
}

void ParticleSystem::setNumParticles(int numPart) {
	this->numParticles = numPart;
}
