#include "HumanSystem.h"

HumanSystem::HumanSystem(const std::string& name) :
	BaseSystem(name)
{
	this->setHumanScale(2, 4, 2);
	this->setHumanPosition(0, -3, -7);
	this->initPHand();

	D2ArrayCopy(1, 7, this->theta, this->thetaRest);

	simPHand[0] = 999;
	simPHand[1] = 999;
	simPHand[2] = 999;
	simPHand[3] = 1;

}	// HumanSystem

void HumanSystem::getState(HumanSystem* h) {
	h = this;
}	// HumanSystem::getState

void HumanSystem::setState(HumanSystem* h) {
	*this = *h;
}	// HumanSystem::setState

void HumanSystem::reset(double time) {
	if (this->splineLoaded) D2ArrayCopy(1, 7, this->theta, this->thetaReady);
	else D2ArrayCopy(1, 7, this->theta, this->thetaRest);

	this->setHumanPosition(this->torsoPos[0], this->torsoPos[1], this->torsoPos[2]);
	this->initPHand();

	this->splineReached = false;
	this->simReset = true;
}

int HumanSystem::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "position") == 0) {
		if (argc == 4) {
			this->setHumanPosition(atof(argv[1]), atof(argv[2]), atof(argv[3]));
		}
	}
	else {
		animTcl::OutputMessage("Command not found");
		return TCL_ERROR;
	}

	glutPostRedisplay();
	return TCL_OK;
}	// HumanSystem::command

void HumanSystem::display(GLenum mode) {
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glMatrixMode(GL_MODELVIEW);

	glColor3f(0.5, 0, 0);	// dark red


	// PUSH TORSO MATRIX
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

		this->yRotate(this->mat, 180);
		this->setGlMat(this->mat);
		glMultMatrixd(this->glMat);
		this->translate(this->mat, this->torsoPos[0], this->torsoPos[1], this->torsoPos[2]);
		this->setGlMat(this->mat);
		glMultMatrixd(this->glMat);		// T_root

		// SCALE AND DRAW TORSO
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			this->scale(this->mat, this->torsoScl[0], this->torsoScl[1], this->torsoScl[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);		// torso scaling
			glutSolidSphere(1.0, 20, 20);	// draw torso
		glPopMatrix();
		glPopAttrib();

		/***********************************************************************************************/

		// PUSH RIGHT SHOULDER MATRIX
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			
			this->translate(this->mat, this->rightShoulder[0], this->rightShoulder[1], this->rightShoulder[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// T_sh
			this->zRotate(this->mat, this->theta[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// R_sh_z(theta_3)
			this->yRotate(this->mat, this->theta[1]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// R_sh_y(theta_2)
			this->xRotate(this->mat, this->theta[0]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// R_sh_x(theta_1)

			// SCALE AND DRAW RIGHT UPPER ARM
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);				
				this->translate(this->mat, this->rightUpperArmPos[0], this->rightUpperArmPos[1], this->rightUpperArmPos[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// translate right upper arm
				this->scale(this->mat, this->rightUpperArmScl[0], this->rightUpperArmScl[1], this->rightUpperArmScl[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// right upper arm scaling
				glutSolidSphere(1.0, 20, 20);		// draw right upper arm
			glPopMatrix();
			glPopAttrib();

			// PUSH RIGHT ELBOW MATRIX
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);

				this->translate(this->mat, this->rightElbow[0], this->rightElbow[1], this->rightElbow[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// T_el
				this->yRotate(this->mat, this->theta[4]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// R_el_y(theta_5)
				this->xRotate(this->mat, this->theta[3]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// R_el_x(theta_4)
				
				
				// SCALE AND DRAW RIGHT LOWER ARM
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);
					this->translate(this->mat, this->rightLowerArmPos[0], this->rightLowerArmPos[1], this->rightLowerArmPos[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// translate right lower arm
					this->scale(this->mat, this->rightLowerArmScl[0], this->rightLowerArmScl[1], this->rightLowerArmScl[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// right lower arm scaling
					glutSolidSphere(1.0, 20, 20);		// draw right upper arm
				glPopMatrix();
				glPopAttrib();

				// PUSH RIGHT WRIST MATRIX
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);

					this->translate(this->mat, this->rightWrist[0], this->rightWrist[1], this->rightWrist[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);					// T_wr
					this->zRotate(this->mat, this->theta[5]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);					// R_wr_z(theta_6)
					this->yRotate(this->mat, this->theta[6]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);					// R_wr_y(theta_7)

					// SCALE AND DRAW RIGHT HAND
					glPushMatrix();
					glPushAttrib(GL_ALL_ATTRIB_BITS);
						this->translate(this->mat, this->rightHandPos[0], this->rightHandPos[1], this->rightHandPos[2]);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);			// translate right hand
						this->scale(this->mat, this->rightHandScl[0], this->rightHandScl[1], this->rightHandScl[2]);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);			// right hand scaling
						glutSolidSphere(1.0, 20, 20);		// draw right hand
					glPopMatrix();
					glPopAttrib();

				// POP RIGHT WRIST MATRIX
				glPopMatrix();
				glPopAttrib();

			// POP RIGHT ELBOW MATRIX
			glPopMatrix();
			glPopAttrib();

		// POP RIGHT SHOULDER MATRIX
		glPopMatrix();
		glPopAttrib();

		/***********************************************************************************************/

		// PUSH LEFT SHOULDER MATRIX
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

			this->translate(this->mat, this->leftShoulder[0], this->leftShoulder[1], this->leftShoulder[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// T_sh
			if (this->splineLoaded) {
				this->zRotate(this->mat, -75);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// R_sh_z
				this->yRotate(this->mat, 0);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// R_sh_y
				this->xRotate(this->mat, 0);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// R_sh_x
			}

			// SCALE AND DRAW LEFT UPPER ARM
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
				this->translate(this->mat, this->leftUpperArmPos[0], this->leftUpperArmPos[1], this->leftUpperArmPos[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// translate left upper arm
				this->scale(this->mat, this->leftUpperArmScl[0], this->leftUpperArmScl[1], this->leftUpperArmScl[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// left upper arm scaling
				glutSolidSphere(1.0, 20, 20);		// draw left upper arm
			glPopMatrix();
			glPopAttrib();

			// PUSH LEFT ELBOW MATRIX
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);

				this->translate(this->mat, this->leftElbow[0], this->leftElbow[1], this->leftElbow[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);					// T_el
				if (this->splineLoaded) {
					this->yRotate(this->mat, -10);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);					// R_el_y
					this->xRotate(this->mat, 0);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);					// R_el_x
				}

				// SCALE AND DRAW LEFT LOWER ARM
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);
					this->translate(this->mat, this->leftLowerArmPos[0], this->leftLowerArmPos[1], this->leftLowerArmPos[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// translate left lower arm
					this->scale(this->mat, this->leftLowerArmScl[0], this->leftLowerArmScl[1], this->leftLowerArmScl[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// left lower arm scaling
					glutSolidSphere(1.0, 20, 20);		// draw left upper arm
				glPopMatrix();
				glPopAttrib();

				// PUSH LEFT WRIST MATRIX
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);

					this->translate(this->mat, this->leftWrist[0], this->leftWrist[1], this->leftWrist[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);					// T_wr
					if (this->splineLoaded) {
						this->zRotate(this->mat, 0);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);				// R_wr_z
						this->yRotate(this->mat, 0);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);				// R_wr_y
					}

					// SCALE AND DRAW LEFT HAND
					glPushMatrix();
					glPushAttrib(GL_ALL_ATTRIB_BITS);
						this->translate(this->mat, this->leftHandPos[0], this->leftHandPos[1], this->leftHandPos[2]);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);			// translate left hand
						this->scale(this->mat, this->leftHandScl[0], this->leftHandScl[1], this->leftHandScl[2]);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);			// left hand scaling
						glutSolidSphere(1.0, 20, 20);		// draw left hand
					glPopMatrix();
					glPopAttrib();

				// POP LEFT WRIST MATRIX
				glPopMatrix();
				glPopAttrib();

			// POP LEFT ELBOW MATRIX
			glPopMatrix();
			glPopAttrib();

		// POP LEFT SHOULDER MATRIX
		glPopMatrix();
		glPopAttrib();

		/***********************************************************************************************/

		// PUSH RIGHT HIP MATRIX
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

			this->translate(this->mat, this->rightHip[0], this->rightHip[1], this->rightHip[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);				// translate hip
			this->xRotate(this->mat, 0);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// R_hip_x

			// SCALE AND DRAW RIGHT UPPER LEG
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
				this->translate(this->mat, this->rightUpperLegPos[0], this->rightUpperLegPos[1], this->rightUpperLegPos[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// translate right upper leg
				this->scale(this->mat, this->rightUpperLegScl[0], this->rightUpperLegScl[1], this->rightUpperLegScl[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// right upper leg scaling
				glutSolidSphere(1.0, 20, 20);		// draw right upper leg
			glPopMatrix();
			glPopAttrib();

			// PUSH RIGHT KNEE MATRIX
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);

				this->translate(this->mat, this->rightKnee[0], this->rightKnee[1], this->rightKnee[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);				// translate knee
				this->xRotate(this->mat, 0);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);				// R_knee_x

				// SCALE AND DRAW RIGHT LOWER LEG
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);
					this->translate(this->mat, this->rightLowerLegPos[0], this->rightLowerLegPos[1], this->rightLowerLegPos[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// translate right lower leg
					this->scale(this->mat, this->rightLowerLegScl[0], this->rightLowerLegScl[1], this->rightLowerLegScl[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// right lower leg scaling
					glutSolidSphere(1.0, 20, 20);		// draw right lower leg
				glPopMatrix();
				glPopAttrib();

				// PUSH RIGHT ANKLE MATRIX
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);

					this->translate(this->mat, this->rightAnkle[0], this->rightAnkle[1], this->rightAnkle[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);				// translate ankle
					this->xRotate(this->mat, 0);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);				// R_ankle_x

					// SCALE AND DRAW RIGHT FOOT
					glPushMatrix();
					glPushAttrib(GL_ALL_ATTRIB_BITS);
						this->translate(this->mat, this->rightFootPos[0], this->rightFootPos[1], this->rightFootPos[2]);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);			// translate right foot
						this->scale(this->mat, this->rightFootScl[0], this->rightFootScl[1], this->rightFootScl[2]);
						this->setGlMat(this->mat);
						glMultMatrixd(this->glMat);			// right foot scaling
						glutSolidSphere(1.0, 20, 20);		// draw right foot
					glPopMatrix();
					glPopAttrib();

				// POP RIGHT ANKLE MATRIX
				glPopMatrix();
				glPopAttrib();

			// POP RIGHT KNEE MATRIX
			glPopMatrix();
			glPopAttrib();

		// POP RIGHT HIP MATRIX
		glPopMatrix();
		glPopAttrib();

		/***********************************************************************************************/

		// PUSH LEFT HIP MATRIX
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

			this->translate(this->mat, this->leftHip[0], this->leftHip[1], this->leftHip[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);				// translate hip
			this->xRotate(this->mat, 0);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);					// R_hip_x

			// SCALE AND DRAW LEFT UPPER LEG
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);
				this->translate(this->mat, this->leftUpperLegPos[0], this->leftUpperLegPos[1], this->leftUpperLegPos[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// translate left upper leg
				this->scale(this->mat, this->leftUpperLegScl[0], this->leftUpperLegScl[1], this->leftUpperLegScl[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);			// left upper leg scaling
				glutSolidSphere(1.0, 20, 20);		// draw left upper leg
			glPopMatrix();
			glPopAttrib();

			// PUSH LEFT KNEE MATRIX
			glPushMatrix();
			glPushAttrib(GL_ALL_ATTRIB_BITS);

				this->translate(this->mat, this->leftKnee[0], this->leftKnee[1], this->leftKnee[2]);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);				// translate knee
				this->xRotate(this->mat, 0);
				this->setGlMat(this->mat);
				glMultMatrixd(this->glMat);				// R_knee_x

				// SCALE AND DRAW LEFT LOWER LEG
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);
					this->translate(this->mat, this->leftLowerLegPos[0], this->leftLowerLegPos[1], this->leftLowerLegPos[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// translate left lower leg
					this->scale(this->mat, this->leftLowerLegScl[0], this->leftLowerLegScl[1], this->leftLowerLegScl[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// left lower leg scaling
					glutSolidSphere(1.0, 20, 20);		// draw left lower leg
				glPopMatrix();
				glPopAttrib();

				// PUSH LEFT ANKLE MATRIX
				glPushMatrix();
				glPushAttrib(GL_ALL_ATTRIB_BITS);

					this->translate(this->mat, this->leftAnkle[0], this->leftAnkle[1], this->leftAnkle[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);				// translate ankle
					this->xRotate(this->mat, 0);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);				// R_ankle_x

					// SCALE AND DRAW LEFT FOOT
					glPushMatrix();
					glPushAttrib(GL_ALL_ATTRIB_BITS);
					this->translate(this->mat, this->leftFootPos[0], this->leftFootPos[1], this->leftFootPos[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// translate left foot
					this->scale(this->mat, this->leftFootScl[0], this->leftFootScl[1], this->leftFootScl[2]);
					this->setGlMat(this->mat);
					glMultMatrixd(this->glMat);			// left foot scaling
					glutSolidSphere(1.0, 20, 20);		// draw left foot
					glPopMatrix();
					glPopAttrib();

				// POP LEFT ANKLE MATRIX
				glPopMatrix();
				glPopAttrib();

			// POP LEFT KNEE MATRIX
			glPopMatrix();
			glPopAttrib();

		// POP LEFT HIP MATRIX
		glPopMatrix();
		glPopAttrib();

		/***********************************************************************************************/

		// PUSH HEAD MATRIX
		glPushMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

			this->translate(this->mat, this->headPos[0], this->headPos[1], this->headPos[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);				// translate head
			this->scale(this->mat, this->headScl[0], this->headScl[1], this->headScl[2]);
			this->setGlMat(this->mat);
			glMultMatrixd(this->glMat);			// head scaling
			glutSolidSphere(1.0, 20, 20);		// draw head

		// POP HEAD MATRIX
		glPopMatrix();
		glPopAttrib();

		/***********************************************************************************************/

	// POP TORSO MATRIX
	glPopMatrix();
	glPopAttrib();


	// BLACKBOARD
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(0, 0.3, 0);	// dark green
		glBegin(GL_POLYGON);
		glVertex3f(6.0, 6.0, -0.1);
		glVertex3f(-6.0, 6.0, -0.1);
		glVertex3f(-6.0, -6.0, -0.1);
		glVertex3f(6.0, -6.0, -0.1);
		glEnd();
	glPopMatrix();
	glPopAttrib();

	// BLACKBOARD EDGE
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(0.3, 0.3, 0);		// brown
		glBegin(GL_POLYGON);
		glVertex3f(6.5, 6.5, -0.2);
		glVertex3f(-6.5, 6.5, -0.2);
		glVertex3f(-6.5, -6.5, -0.2);
		glVertex3f(6.5, -6.5, -0.2);
		glEnd();
	glPopMatrix();
	glPopAttrib();

	// FRONT WALL
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(1, 0.7, 0.2);	// mustard yellow
		glBegin(GL_POLYGON);
		glVertex3f(30.0, 15.0, -0.3);
		glVertex3f(-30.0, 15.0, -0.3);
		glVertex3f(-30.0, -16.5, -0.3);
		glVertex3f(30.0, -16.5, -0.3);
		glEnd();
	glPopMatrix();
	glPopAttrib();

	// RIGHT WALL
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(1, 0.7, 0.2);	// mustard yellow
		glBegin(GL_POLYGON);
		glVertex3f(30.0, 15.0, -0.3);
		glVertex3f(30.0, 15.0, 20.0);
		glVertex3f(30.0, -16.5, 20.0);
		glVertex3f(30.0, -16.5, -0.3);
		glEnd();
	glPopMatrix();
	glPopAttrib();

	// LEFT WALL
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(1, 0.7, 0.2);	// mustard yellow
		glBegin(GL_POLYGON);
		glVertex3f(-30.0, 15.0, -0.3);
		glVertex3f(-30.0, 15.0, 20.0);
		glVertex3f(-30.0, -16.5, 20.0);
		glVertex3f(-30.0, -16.5, -0.3);
		glEnd();
	glPopMatrix();
	glPopAttrib();

	// FLOOR
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(1, 0.6, 0);	// mustard yellow
		glBegin(GL_POLYGON);
		glVertex3f(-30.0, -16.5, -0.3);
		glVertex3f(30.0, -16.5, -0.3);
		glVertex3f(30.0, -16.5, 20.0);
		glVertex3f(-30.0, -16.5, 20.0);
		glEnd();
	glPopMatrix();
	glPopAttrib();

	// END EFFECTOR
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3d(0, 0, 0);		// black
		glRotated(180, 0, 1, 0);
		glTranslated(simPHand[0], simPHand[1], simPHand[2]);
		glScaled(0.1, 0.1, 0.1);
		glutSolidSphere(1.0, 20, 20);
	glPopMatrix();
	glPopAttrib();
	
}



double HumanSystem::degToRad(double deg) {
	return (deg * (M_PI / 180));
}

double HumanSystem::radToDeg(double rad) {
	return (rad * (180 / M_PI));
}

void HumanSystem::setHumanPosition(double x, double y, double z) {
	// torso
	this->setPosition(this->torsoPos, x, y, z);

	// head
	this->setPosition(this->headPos, 0, this->torsoScl[1] * 1.28, 0);

	// right arm
	this->setPosition(this->rightShoulder, -this->torsoScl[0] * 0.7, this->torsoScl[1] * 0.75, 0);
	this->setPosition(this->rightUpperArmPos, -this->torsoScl[0] * 1.1, 0, 0);
	this->setPosition(this->rightElbow, -this->torsoScl[0] * 2.3, 0, 0);
	this->setPosition(this->rightLowerArmPos, -this->torsoScl[0] * 1.1, 0, 0);
	this->setPosition(this->rightWrist, -this->torsoScl[0] * 2.2, 0, 0);
	this->setPosition(this->rightHandPos, -this->torsoScl[0] * 0.5, 0, 0);

	// left arm
	D2ArrayCopy(1, 3, this->leftShoulder, this->rightShoulder);
	this->leftShoulder[0] *= -1;
	D2ArrayCopy(1, 3, this->leftUpperArmPos, this->rightUpperArmPos);
	this->leftUpperArmPos[0] *= -1;
	D2ArrayCopy(1, 3, this->leftElbow, this->rightElbow);
	this->leftElbow[0] *= -1;
	D2ArrayCopy(1, 3, this->leftLowerArmPos, this->rightLowerArmPos);
	this->leftLowerArmPos[0] *= -1;
	D2ArrayCopy(1, 3, this->leftWrist, this->rightWrist);
	this->leftWrist[0] *= -1;
	D2ArrayCopy(1, 3, this->leftHandPos, this->rightHandPos);
	this->leftHandPos[0] *= -1;

	// right leg
	this->setPosition(this->rightHip, -this->torsoScl[0] * 0.45, -this->torsoScl[1] * 0.95, 0);
	this->setPosition(this->rightUpperLegPos, 0, -this->torsoScl[1] * 0.5, 0);
	this->setPosition(this->rightKnee, 0, -this->torsoScl[1] * 1.1, 0);
	this->setPosition(this->rightLowerLegPos, 0, -this->torsoScl[1] * 0.55, 0);
	this->setPosition(this->rightAnkle, 0, -this->torsoScl[1] * 1.1, 0);
	this->setPosition(this->rightFootPos, 0, -this->torsoScl[1] * 0.1, this->torsoScl[2] * 0.1);

	// left leg
	D2ArrayCopy(1, 3, this->leftHip, this->rightHip);
	this->leftHip[0] *= -1;
	D2ArrayCopy(1, 3, this->leftUpperLegPos, this->rightUpperLegPos);
	this->leftUpperLegPos[0] *= -1;
	D2ArrayCopy(1, 3, this->leftKnee, this->rightKnee);
	this->leftKnee[0] *= -1;
	D2ArrayCopy(1, 3, this->leftLowerLegPos, this->rightLowerLegPos);
	this->leftLowerLegPos[0] *= -1;
	D2ArrayCopy(1, 3, this->leftAnkle, this->rightAnkle);
	this->leftAnkle[0] *= -1;
	D2ArrayCopy(1, 3, this->leftFootPos, this->rightFootPos);
	this->leftFootPos[0] *= -1;
	
}

void HumanSystem::setPosition(Position pos, double x, double y, double z) {
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
}

void HumanSystem::setHumanScale(double x, double y, double z) {
	// torso
	this->setScale(this->torsoScl, x, y, z);

	// head
	this->setScale(this->headScl, this->torsoScl[0] * 0.65, this->torsoScl[1] * 0.305, this->torsoScl[2] * 0.65);

	// right arm
	this->setScale(this->rightUpperArmScl, this->torsoScl[0] * 1.2, this->torsoScl[1] * 0.14, this->torsoScl[2] * 0.32);
	D2ArrayCopy(1, 3, this->rightLowerArmScl, this->rightUpperArmScl);
	this->setScale(this->rightHandScl, this->headScl[0] * 0.75, this->headScl[1] * 0.4, this->headScl[2] * 0.4);

	// left arm
	D2ArrayCopy(1, 3, this->leftUpperArmScl, this->rightUpperArmScl);
	D2ArrayCopy(1, 3, this->leftLowerArmScl, this->rightLowerArmScl);
	D2ArrayCopy(1, 3, this->leftHandScl, this->rightHandScl);

	// right leg
	this->setScale(this->rightUpperLegScl, this->torsoScl[0] * 0.32, this->torsoScl[1] * 0.6, this->torsoScl[2] * 0.32);
	D2ArrayCopy(1, 3, this->rightLowerLegScl, this->rightUpperLegScl);
	this->setScale(this->rightFootScl, this->torsoScl[0] * 0.4, this->torsoScl[1] * 0.1, this->torsoScl[2] * 0.4);

	// left leg
	this->setScale(this->leftUpperLegScl, this->torsoScl[0] * 0.32, this->torsoScl[1] * 0.6, this->torsoScl[2] * 0.32);
	D2ArrayCopy(1, 3, this->leftLowerLegScl, this->leftUpperLegScl);
	this->setScale(this->leftFootScl, this->torsoScl[0] * 0.4, this->torsoScl[1] * 0.1, this->torsoScl[2] * 0.4);

}

void HumanSystem::setScale(Scale scl, double x, double y, double z) {
	scl[0] = x;
	scl[1] = y;
	scl[2] = z;
}

void HumanSystem::setGlMat(Mat4x4 mat) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			this->glMat[i * 4 + j] = mat[j][i];
		}
	}
}

void HumanSystem::initPHand() {
	this->pHand[0] = -this->rightHandScl[0] * 2;
	this->pHand[1] = this->pHand[2] = 0;
	this->pHand[3] = 1;
}

void HumanSystem::translate(Mat4x4 mat, double x, double y, double z) {
	Mat4x4 transl = {
		{1,		0,		0,		x},
		{0,		1,		0,		y},
		{0,		0,		1,		z},
		{0,		0,		0,		1}
	};

	D2ArrayCopy(4, 4, *mat, *transl);
}

void HumanSystem::scale(Mat4x4 mat, double x, double y, double z) {
	Mat4x4 scl = {
		{x,		0,		0,		0},
		{0,		y,		0,		0},
		{0,		0,		z,		0},
		{0,		0,		0,		1}
	};

	D2ArrayCopy(4, 4, *mat, *scl);
}

void HumanSystem::xRotate(Mat4x4 mat, double theta) {

	double rad = degToRad(theta);

	Mat4x4 xRoll = {
		{1,		0,				0,				0},
		{0,		cos(rad),		-sin(rad),		0},
		{0,		sin(rad),		cos(rad),		0},
		{0,		0,				0,				1}
	};

	D2ArrayCopy(4, 4, *mat, *xRoll);
}

void HumanSystem::yRotate(Mat4x4 mat, double theta) {

	double rad = degToRad(theta);

	Mat4x4 yRoll = {
		{cos(rad),		0,		sin(rad),		0},
		{0,				1,		0,				0},
		{-sin(rad),		0,		cos(rad),		0},
		{0,				0,		0,				1}
	};

	D2ArrayCopy(4, 4, *mat, *yRoll);
}

void HumanSystem::zRotate(Mat4x4 mat, double theta) {

	double rad = degToRad(theta);

	Mat4x4 zRoll = {
		{cos(rad),		-sin(rad),		0,		0},
		{sin(rad),		cos(rad),		0,		0},
		{0,				0,				1,		0},
		{0,				0,				0,		1}
	};

	D2ArrayCopy(4, 4, *mat, *zRoll);
}

void HumanSystem::xRotDeriv(Mat4x4 mat, double theta) {

	double rad = degToRad(theta);

	Mat4x4 xRollDeriv = {
		{0,		0,				0,				0},
		{0,		-sin(rad),		-cos(rad),		0},
		{0,		cos(rad),		-sin(rad),		0},
		{0,		0,				0,				0}
	};

	D2ArrayCopy(4, 4, *mat, *xRollDeriv);
}

void HumanSystem::yRotDeriv(Mat4x4 mat, double theta) {

	double rad = degToRad(theta);

	Mat4x4 yRollDeriv = {
		{-sin(rad),		0,		cos(rad),		0},
		{0,				0,		0,				0},
		{-cos(rad),		0,		-sin(rad),		0},
		{0,				0,		0,				0}
	};

	D2ArrayCopy(4, 4, *mat, *yRollDeriv);
}

void HumanSystem::zRotDeriv(Mat4x4 mat, double theta) {

	double rad = degToRad(theta);

	Mat4x4 zRollDeriv = {
		{-sin(rad),		-cos(rad),		0,		0},
		{cos(rad),		-sin(rad),		0,		0},
		{0,				0,				0,		0},
		{0,				0,				0,		0}
	};

	D2ArrayCopy(4, 4, *mat, *zRollDeriv);
}

void HumanSystem::transformPointOrVec(Mat4x4 mat, double* ptOrVec) {
	Point cpy;

	D2ArrayCopy(1, 4, cpy, ptOrVec);
	multArray(ptOrVec, *mat, cpy, 4, 4, 1);
}
