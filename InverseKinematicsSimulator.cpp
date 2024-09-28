#include "InverseKinematicsSimulator.h"

InverseKinematicsSimulator::InverseKinematicsSimulator(const std::string& name, BaseSystem* target1, BaseSystem* target2) :
	BaseSimulator(name),
	human((HumanSystem*)target1),
	herm((Hermite*)target2)
{
	setVector(this->threshold, 0.08, 0.08, 0.08);

	this->targetP[0] = this->targetP[1] = this->targetP[2] = 0;
	this->targetP[3] = 1;

	this->u = 0;

}	// InverseKinematicsSimulator

InverseKinematicsSimulator::~InverseKinematicsSimulator()
{
}	// InverseKinematicsSimulator::~InverseKinematicsSimulator()

int InverseKinematicsSimulator::init(double time) {
	return 0;
}	// InverseKinematicsSimulator::init

int InverseKinematicsSimulator::step(double time) {
	if (human->simReset) {
		this->u = 0;
		this->p = herm->getIntermediatePoint(0);
		this->targetP[0] = -p[0];
		this->targetP[1] = p[1];
		this->targetP[2] = p[2];

		human->splineReached = false;
		human->simReset = false;
	}

	IKSolver();
	return TCL_OK;
}	// InverseKinematicsSimulator::step

int InverseKinematicsSimulator::command(int argc, myCONST_SPEC char** argv) {

	if (argc < 1) {
		animTcl::OutputMessage("simulator %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0) {
		if (argc == 2) {
			herm->loadFromFile2D(argv[1]);
			human->splineLoaded = true;
			D2ArrayCopy(1, 7, human->theta, human->thetaReady);
			this->u = 0;
			human->splineReached = false;
			this->p = herm->getIntermediatePoint(0);
			this->targetP[0] = -p[0];
			this->targetP[1] = p[1];
			this->targetP[2] = p[2];
		}
		else {
			animTcl::OutputMessage("Usage: read <file_name> ");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "threshold") == 0) {
		if (argc == 4) {
			threshold[0] = atof(argv[1]);
			threshold[1] = atof(argv[2]);
			threshold[2] = atof(argv[3]);
		}
		else {
			animTcl::OutputMessage("Usage: threshold <x> <y> <z>");
			return TCL_ERROR;
		}
	}
	else {
		animTcl::OutputMessage("Command not found");
		return TCL_ERROR;
	}

	return TCL_OK;
}

void InverseKinematicsSimulator::IKSolve() {

	Array2D<double> A(3, 3);	// J * J^T
	Array2D<double> invA(3, 3);	// (J * J^T)^-1
	Array2D<double> b(3, 1);	// delta x
	Array2D<double> x(3, 1);	// beta
	Array2D<double> I(3, 3);

	double jTransp[7][3];
	double jTimesjTra[3][3];
	double beta[3];

	transpArray(*jTransp, *this->jacobian, 3, 7);
	multArray(*jTimesjTra, *this->jacobian, *jTransp, 3, 7, 3);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A[i][j] = jTimesjTra[i][j];
		}
	}

	subtractArray(deltaX, pTargetP, currentP, 1, 4);

	b[0][0] = deltaX[0];
	b[1][0] = deltaX[1];
	b[2][0] = deltaX[2];

	JAMA::LU<double> lu(A);

	x = lu.solve(b);

	beta[0] = x[0][0];
	beta[1] = x[1][0];
	beta[2] = x[2][0];
	
	multArray(this->deltaTheta, *jTransp, beta, 7, 3, 1);

	for (int k = 0; k < 7; k++) {
		human->theta[k] += this->deltaTheta[k];
	}
}

void InverseKinematicsSimulator::IKSolver() {
	this->endEffInWorldCS();	// currentP (End Effector in World CS)

	D2ArrayCopy(1, 4, human->simPHand, this->currentP);

	this->err[0] = this->targetP[0] - this->currentP[0];	// err = targetP - currentP
	this->err[1] = this->targetP[1] - this->currentP[1];
	this->err[2] = this->targetP[2] - this->currentP[2];

	if (VecLength(this->err) >= VecLength(this->threshold)) {

		this->calcJ();

		if (!human->splineReached) {
			this->pTargetP[0] = 0.1 * this->err[0] + this->currentP[0];		// pTargetP = t * err + currentP
			this->pTargetP[1] = 0.1 * this->err[1] + this->currentP[1];
			this->pTargetP[2] = 0.1 * this->err[2] + this->currentP[2];
			this->pTargetP[3] = 1;
		}
		else {
			this->u += this->du;
			if (u >= 1.0) u = 0.9999;
			this->p = herm->getIntermediatePoint(u);
			this->pTargetP[0] = -p[0];
			this->pTargetP[1] = p[1];
			this->pTargetP[2] = p[2];
			this->pTargetP[3] = 1;
		}

		this->IKSolve();

		this->endEffInWorldCS();	// currentP (End Effector in World CS)

		D2ArrayCopy(1, 4, human->simPHand, this->currentP);

		this->err[0] = this->targetP[0] - this->currentP[0];	// err = targetP - currentP
		this->err[1] = this->targetP[1] - this->currentP[1];
		this->err[2] = this->targetP[2] - this->currentP[2];

	}
	else if (!human->splineReached) {
		human->splineReached = true;
		this->p = herm->getIntermediatePoint(0.9999);
		this->targetP[0] = -p[0];
		this->targetP[1] = p[1];
		this->targetP[2] = p[2];
	}
	else {
		human->splineReached = false;
		this->u = 0;
		this->p = herm->getIntermediatePoint(0);
		this->targetP[0] = -p[0];
		this->targetP[1] = p[1];
		this->targetP[2] = p[2];
	}

}

void InverseKinematicsSimulator::calcJ() {

	Mat4x4 mat;
	double jacCol[4];

	for (int i = 0; i < 7; i++) {

		// calc current end eff point in Wrist CS
		human->initPHand();

		// initialize column i of Jacobian to pHand but with last element = 0
		D2ArrayCopy(1, 4, jacCol, human->pHand);

		// vec_A = R_z( theta_6 ) * pHand  OR  R_z ' ( theta_6 ) * pHand
		if (i == 5) human->zRotDeriv(mat, human->theta[5]);
		else human->zRotate(mat, human->theta[5]);
		human->transformPointOrVec(mat, jacCol);

		// vec_B = R_y( theta_7 ) * vec_A  OR  R_y ' ( theta_7 ) * vec_A
		if (i == 6) human->yRotDeriv(mat, human->theta[6]);
		else human->yRotate(mat, human->theta[6]);
		human->transformPointOrVec(mat, jacCol);

		// vec_C = T_wr * vec_B
		human->translate(mat, human->rightWrist[0], human->rightWrist[1], human->rightWrist[2]);
		human->transformPointOrVec(mat, jacCol);

		// vec_D = R_x( theta_4 ) * vec_C  OR  R_x ' ( theta_4 ) * vec_C
		if (i == 3) human->xRotDeriv(mat, human->theta[3]);
		else human->xRotate(mat, human->theta[3]);
		human->transformPointOrVec(mat, jacCol);

		// vec_E = R_y( theta_5 ) * vec_D  OR  R_y ' ( theta_5 ) * vec_D
		if (i == 4) human->yRotDeriv(mat, human->theta[4]);
		else human->yRotate(mat, human->theta[4]);
		human->transformPointOrVec(mat, jacCol);

		// vec_F = T_el * vec_E
		human->translate(mat, human->rightElbow[0], human->rightElbow[1], human->rightElbow[2]);
		human->transformPointOrVec(mat, jacCol);

		// vec_G = R_x( theta_1 ) * vec_G  OR  R_x ' ( theta_1 ) * vec_G
		if (i == 0) human->xRotDeriv(mat, human->theta[0]);
		else human->xRotate(mat, human->theta[0]);
		human->transformPointOrVec(mat, jacCol);

		// vec_H = R_y( theta_2 ) * vec_G  OR  R_y ' ( theta_2 ) * vec_G
		if (i == 1) human->yRotDeriv(mat, human->theta[1]);
		else human->yRotate(mat, human->theta[1]);
		human->transformPointOrVec(mat, jacCol);

		// vec_I = R_z( theta_3 ) * vec_H  OR  R_z ' ( theta_3 ) * vec_H
		if (i == 2) human->zRotDeriv(mat, human->theta[2]);
		else human->zRotate(mat, human->theta[2]);
		human->transformPointOrVec(mat, jacCol);

		// vec_J = T_sh * vec_I
		human->translate(mat, human->rightShoulder[0], human->rightShoulder[1], human->rightShoulder[2]);
		human->transformPointOrVec(mat, jacCol);

		// P_world = vec_K = T_root * vec_J
		human->translate(mat, human->torsoPos[0], human->torsoPos[1], human->torsoPos[2]);
		human->transformPointOrVec(mat, jacCol);

		this->jacobian[0][i] = jacCol[0];
		this->jacobian[1][i] = jacCol[1];
		this->jacobian[2][i] = jacCol[2];

	}
}

void InverseKinematicsSimulator::endEffInWorldCS() {
	Mat4x4 mat;

	D2ArrayCopy(1, 4, this->currentP, human->pHand);

	human->zRotate(mat, human->theta[5]);
	human->transformPointOrVec(mat, this->currentP);

	human->yRotate(mat, human->theta[6]);
	human->transformPointOrVec(mat, this->currentP);

	human->translate(mat, human->rightWrist[0], human->rightWrist[1], human->rightWrist[2]);
	human->transformPointOrVec(mat, this->currentP);

	human->xRotate(mat, human->theta[3]);
	human->transformPointOrVec(mat, this->currentP);

	human->yRotate(mat, human->theta[4]);
	human->transformPointOrVec(mat, this->currentP);

	human->translate(mat, human->rightElbow[0], human->rightElbow[1], human->rightElbow[2]);
	human->transformPointOrVec(mat, this->currentP);

	human->xRotate(mat, human->theta[0]);
	human->transformPointOrVec(mat, this->currentP);

	human->yRotate(mat, human->theta[1]);
	human->transformPointOrVec(mat, this->currentP);

	human->zRotate(mat, human->theta[2]);
	human->transformPointOrVec(mat, this->currentP);

	human->translate(mat, human->rightShoulder[0], human->rightShoulder[1], human->rightShoulder[2]);
	human->transformPointOrVec(mat, this->currentP);

	human->translate(mat, human->torsoPos[0], human->torsoPos[1], human->torsoPos[2]);
	human->transformPointOrVec(mat, this->currentP);

}
