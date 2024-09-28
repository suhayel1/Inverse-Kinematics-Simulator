#include "ParticleSimulator.h"

double ParticleSimulator::dt = 0.01;

ParticleSimulator::ParticleSimulator(const std::string& name, BaseSystem* target) :
	BaseSimulator(name),
	pObject((ParticleSystem*)target),
	grKS(0),
	grKD(0),
	kDrag(0)
{
	setVector(grav, 0, -9.8, 0);
	setVector(norm, 0, 1, 0);
	setVector(point, 0, 0, 0);
	zeroVector(mousePos);
	lmbIsHeldDown = false;

	mouseIndex = 1000;
	pObject->getSpring(mouseIndex).setI(mouseIndex);
	pObject->getSpring(mouseIndex).setKS(100);
	pObject->getSpring(mouseIndex).setKD(10);
	pObject->getSpring(mouseIndex).setRL(0);
}	// ParticleSimulator

ParticleSimulator::~ParticleSimulator()
{
}	// ParticleSimulator::~ParticleSimulator

int ParticleSimulator::step(double time) {
	
	// Reset forces
	for (int h = 0; h < pObject->getNumParticles(); h++) {
		pObject->getParticle(h).setForce(0, 0, 0);
	}

	// Calculate and add spring/damp forces to force of particle i
	for (int i = 0; i < pObject->getSprInUse(); i++) {
		objSpr = &(pObject->getSpring(i));
		objPart1 = &(pObject->getParticle(pObject->getSpring(i).getI()));
		objPart2 = &(pObject->getParticle(pObject->getSpring(i).getJ()));

		// Set velocity of particle i and/or j to 0 if fixed
		if (objPart1->getIsFixed()) objPart1->setVel(0, 0, 0);
		if (objPart2->getIsFixed()) objPart2->setVel(0, 0, 0);

		objSpr->springForce(*(objPart1), *(objPart2));
		objSpr->damperForce(*(objPart1), *(objPart2));
	}

	
	// Mouse interaction:
	
	VecCopy(pObject->getParticle(mouseIndex).getPos(), mousePos);
	pObject->getParticle(mouseIndex).setForce(0, 0, 0);

	objSpr = &(pObject->getSpring(mouseIndex));

	if (lmbIsHeldDown) {
		
		for (int k = 0; k < pObject->getNumParticles(); k++) {
			if (k == 0 && objSpr->getJ() != -1) break;
			else if (pObject->getParticle(k).getIsFixed()) continue;
			else {
				VecSubtract(diff, pObject->getParticle(mouseIndex).getPos(), pObject->getParticle(k).getPos());
				diff[2] = 0;
				if (objSpr->getJ() == -1) {
					objSpr->setJ(k);
					minDistFromPtr = VecLength(diff);
				}
				else {
					distFromPtr = VecLength(diff);

					if (distFromPtr < minDistFromPtr || (distFromPtr == minDistFromPtr && pObject->getParticle(k).getPos()[2] > pObject->getParticle(objSpr->getJ()).getPos()[2])) {
						objSpr->setJ(k);
						minDistFromPtr = distFromPtr;
					}
				}
			}
		}

		pObject->getParticle(mouseIndex).getPos()[2] = pObject->getParticle(objSpr->getJ()).getPos()[2];

		objSpr->springForce(pObject->getParticle(objSpr->getI()), pObject->getParticle(objSpr->getJ()));
		objSpr->damperForce(pObject->getParticle(objSpr->getI()), pObject->getParticle(objSpr->getJ()));
		
	}
	else objSpr->setJ(-1);
	

	for (int j = 0; j < pObject->getNumParticles(); j++) {

		objPart1 = &(pObject->getParticle(j));

		if (objPart1->getIsFixed()) continue;

		// Calculate and add rest of forces for each particle (drag + grav + coll)
		totalForce(j);

		// Calculate acceleration
		VecCopy(objPart1->getAcc(), objPart1->getForce());
		VecScale(objPart1->getAcc(), 1.0 / objPart1->getMass());

		// Calculate new position and velocity based on selected integration method

		if (strcmp(integMethod.c_str(), "euler") == 0) {
			eulerPosition(j);
			eulerVelocity(j);
		}
		else if (strcmp(integMethod.c_str(), "symplectic") == 0) {
			eulerVelocity(j);
			eulerPosition(j);
		}
		else if (strcmp(integMethod.c_str(), "verlet") == 0) verlet(j);
		else {
			animTcl::OutputMessage("No suitable integration method found");
			return TCL_ERROR;
		}
	}

	return 0;
}	// ParticleSimulator::step

int ParticleSimulator::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "link") == 0)
	{
		if (argc == 3)
		{
			sysName = argv[1];
			pObject->setNumSprings(atoi(argv[2]));
			pObject->setSprInUse(0);

			if (pObject->getNumSprings() < 1 || pObject->getNumSprings() > 1000) {
				animTcl::OutputMessage("Invalid number of springs");
				return TCL_ERROR;
			}

			//return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: link <sys name> <Number of Springs> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "spring") == 0)
	{
		if (argc == 6)
		{
			Spring* currSpr = &(pObject->getCurrSpr());

			if (pObject->getSprInUse() < pObject->getNumSprings()) {
				int i = atoi(argv[1]);
				int j = atoi(argv[2]);

				currSpr->setI(i);
				currSpr->setJ(j);
				currSpr->setKS(atof(argv[3]));
				currSpr->setKD(atof(argv[4]));
				currSpr->setRL(atof(argv[5]));
				
				if (currSpr->getRL() < 0) {

					Vector dist;

					VecSubtract(dist, pObject->getParticle(i).getPos(), pObject->getParticle(j).getPos());
					
					currSpr->setRL(abs(VecLength(dist)));
				}

				pObject->setSprInUse(pObject->getSprInUse() + 1);
				//return TCL_OK;
			}
			else {
				animTcl::OutputMessage("Max number of springs used");
				return TCL_ERROR;
			}
		}
		else
		{
			animTcl::OutputMessage("Usage: spring <index1> <index2> <ks> <kd> <restlength> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "fix") == 0)
	{
		if (argc == 2)
		{
			if (atoi(argv[1]) >= 0 && atoi(argv[1]) < pObject->getNumParticles())
				pObject->getParticle(atoi(argv[1])).nailParticle();
			else {
				animTcl::OutputMessage("Index out of bounds");
				return TCL_ERROR;
			}
		}
		else
		{
			animTcl::OutputMessage("Usage: fix <index> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "integration") == 0)
	{
		if (argc == 3)
		{
			if (strcmp(argv[1], "euler") != 0 && strcmp(argv[1], "symplectic") != 0 && strcmp(argv[1], "verlet") != 0) {
				animTcl::OutputMessage("Invalid integration method name");
				return TCL_ERROR;
			}

			integMethod = argv[1];

			dt = atof(argv[2]);

		}
		else
		{
			animTcl::OutputMessage("Usage: integration <euler|symplectic|verlet> <time step> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "ground") == 0)
	{
		if (argc == 3)
		{
			grKS = atof(argv[1]);
			grKD = atof(argv[2]);
		}
		else
		{
			animTcl::OutputMessage("Usage: ground <ks> <kd> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "gravity") == 0)
	{
		if (argc == 2)
		{
			grav[1] = atof(argv[1]);
		}
		else
		{
			animTcl::OutputMessage("Usage: gravity <g> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "drag") == 0)
	{
		if (argc == 2)
		{
			if (atof(argv[1]) < 0) {
				animTcl::OutputMessage("Usage: kdrag has to be positive or 0");
				return TCL_ERROR;
			}

			kDrag = atof(argv[1]);
		}
		else
		{
			animTcl::OutputMessage("Usage: drag <kdrag> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "ptr_spring") == 0) {
		if (argc == 3) {
			pObject->getSpring(mouseIndex).setKS(atof(argv[1]));
			pObject->getSpring(mouseIndex).setKD(atof(argv[2]));
		}
		else
		{
			animTcl::OutputMessage("Usage: ptr_spring <ks> <kd>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "reset_sim") == 0) {
		if (argc != 1)
		{
			animTcl::OutputMessage("Usage: reset_sim");
			return TCL_ERROR;
		}

		grKS = 0;
		grKD = 0;
		kDrag = 0;
		setVector(grav, 0, -9.8, 0);
		pObject->setNumSprings(0);
		pObject->setSprInUse(0);
		pObject->setNumParticles(0);
	}
	else {
		animTcl::OutputMessage("Command not found");
		return TCL_ERROR;
	}

	return TCL_OK;
}

void ParticleSimulator::totalForce(int i) {
	Vector dragForce;
	Vector gravForce;
	Vector detectVec;

	Particle* objPart = &(pObject->getParticle(i));

	double detect;

	VecCopy(dragForce, objPart->getVel());
	VecScale(dragForce, -kDrag);
	VecCopy(gravForce, grav);
	VecScale(gravForce, objPart->getMass());

	VecAdd(objPart->getForce(), objPart->getForce(), dragForce);
	VecAdd(objPart->getForce(), objPart->getForce(), gravForce);

	VecSubtract(detectVec, objPart->getPos(), point);
	detect = VecDotProd(detectVec, norm);

	if (detect < DBL_EPSILON) normForce(i, detect);

}

void ParticleSimulator::normForce(int i, double detect) {
	Vector normForce;
	Vector temp;

	double tempDbl1, tempDbl2;

	Particle* objPart = &(pObject->getParticle(i));

	tempDbl2 = VecDotProd(objPart->getVel(), norm);
	tempDbl2 *= grKD;
	VecCopy(temp, norm);
	VecScale(temp, tempDbl2);

	tempDbl1 = -grKS * detect;
	VecCopy(normForce, norm);
	VecScale(normForce, tempDbl1);

	VecSubtract(normForce, normForce, temp);

	VecAdd(objPart->getForce(), objPart->getForce(), normForce);
}

void ParticleSimulator::eulerVelocity(int i) {
	Vector temp;

	Particle* objPart = &(pObject->getParticle(i));

	VecCopy(temp, objPart->getAcc());
	VecScale(temp, dt);
	VecAdd(objPart->getVel(), objPart->getVel(), temp);
}

void ParticleSimulator::eulerPosition(int i) {
	Vector temp;

	Particle* objPart = &(pObject->getParticle(i));

	VecCopy(temp, objPart->getVel());
	VecScale(temp, dt);
	VecAdd(objPart->getPos(), objPart->getPos(), temp);
}

void ParticleSimulator::verlet(int i) {
	Vector a1, a2, a3, newPrevPos;

	Particle* objPart = &(pObject->getParticle(i));

	VecCopy(a1, objPart->getPos());
	VecScale(a1, 2);

	// for first step
	if (VecEq(objPart->getPrevPos(), objPart->getPos()) == 1) {
		// forward Euler:
		eulerPosition(i);
		eulerVelocity(i);

		return;
	}
	else VecCopy(a2, objPart->getPrevPos());

	VecCopy(a3, objPart->getAcc());
	VecScale(a3, dt * dt);

	VecCopy(newPrevPos, objPart->getPos());

	VecSubtract(objPart->getPos(), a1, a2);
	VecAdd(objPart->getPos(), objPart->getPos(), a3);

	VecSubtract(objPart->getVel(), objPart->getPos(), objPart->getPrevPos());
	VecScale(objPart->getVel(), 1.0 / (2.0 * dt));

	VecCopy(objPart->getPrevPos(), newPrevPos);
}

ParticleSystem* ParticleSimulator::getObject() {
	return this->pObject;
}

void ParticleSimulator::setLmbHeldDown(bool lmbHeldDown) {
	this->lmbIsHeldDown = lmbHeldDown;
}

bool ParticleSimulator::getLmbHeldDown() {
	return this->lmbIsHeldDown;
}

void ParticleSimulator::setMousePos(double x, double y, double z) {
	setVector(this->mousePos, x, y, z);
}

Vector& ParticleSimulator::getMousePos() {
	return this->mousePos;
}

void ParticleSimulator::pickFromXYPlane(Vector result, int x, int y) {
	double modelView[16];
	double projection[16];
	int viewport[4];

	double x1, y1, z1, x2, y2, z2;

	glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	y = viewport[3] - y;
	gluUnProject(x, y, 0, modelView, projection, viewport, &x1, &y1, &z1);
	gluUnProject(x, y, 1, modelView, projection, viewport, &x2, &y2, &z2);

	double t = z1 / (z1 - z2);

	result[0] = (1 - t) * x1 + t * x2;
	result[1] = (1 - t) * y1 + t * y2;
	result[2] = 0;

	double z = (1 - t) * z1 + t * z2;
}
