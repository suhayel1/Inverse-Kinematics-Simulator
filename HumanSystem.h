#pragma once
#ifndef HUMAN_SYS_H
#define HUMAN_SYS_H

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>

#include "shared/opengl.h"

#include <util/myMath.h>

typedef double Mat4x4[4][4];
typedef double Position[3];
typedef double Scale[3];
typedef double Point[4];

class HumanSystem : public BaseSystem
{

public:
	HumanSystem(const std::string& name);
	virtual void getState(HumanSystem* h);
	virtual void setState(HumanSystem* h);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { hModel.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&hModel); }
	int command(int argc, myCONST_SPEC char** argv);


	double degToRad(double deg);
	double radToDeg(double rad);

	void setHumanPosition(double x, double y, double z);
	void setPosition(Position pos, double x, double y, double z);
	void setHumanScale(double x, double y, double z);
	void setScale(Scale scl, double x, double y, double z);
	void setGlMat(Mat4x4 mat);
	void initPHand();

	void translate(Mat4x4 mat, double x, double y, double z);
	void scale(Mat4x4 mat, double x, double y, double z);

	void xRotate(Mat4x4 mat, double theta);
	void yRotate(Mat4x4 mat, double theta);
	void zRotate(Mat4x4 mat, double theta);

	void xRotDeriv(Mat4x4 mat, double theta);
	void yRotDeriv(Mat4x4 mat, double theta);
	void zRotDeriv(Mat4x4 mat, double theta);

	void transformPointOrVec(Mat4x4 mat, double* ptOrVec);


	GLMmodel hModel;

	// Body Part Positions
	Position headPos;
	Position torsoPos;
	Position rightShoulder, rightUpperArmPos, rightElbow, rightLowerArmPos, rightWrist, rightHandPos;
	Position leftShoulder, leftUpperArmPos, leftElbow, leftLowerArmPos, leftWrist, leftHandPos;
	Position rightHip, rightUpperLegPos, rightKnee, rightLowerLegPos, rightAnkle, rightFootPos;
	Position leftHip, leftUpperLegPos, leftKnee, leftLowerLegPos, leftAnkle, leftFootPos;

	// Body Part Sizes
	Scale headScl;
	Scale torsoScl;
	Scale rightUpperArmScl;
	Scale rightLowerArmScl;
	Scale rightHandScl;
	Scale leftUpperArmScl;
	Scale leftLowerArmScl;
	Scale leftHandScl;
	Scale rightUpperLegScl;
	Scale rightLowerLegScl;
	Scale rightFootScl;
	Scale leftUpperLegScl;
	Scale leftLowerLegScl;
	Scale leftFootScl;

	// Matrices
	Mat4x4 mat;
	Mat4x4 endEffTransfMat;

	// Matrix for glMultMatrix()
	GLdouble glMat[16];

	// End Effector in Wrist Coordinates
	Point pHand;
	Point simPHand;

	// Joint Angles
	double theta[7];	// {  R_sh_x,  R_sh_y,  R_sh_z,  R_el_x,  R_el_y,  R_wr_z,  R_wr_y }
	double thetaRest[7] = { 0, 0, 0, 0, 0, 0, 0 };
	double thetaReady[7] = { 0, 0, 60, 0, 30, 0, 0 };

	bool splineLoaded = false;
	bool splineReached = false;
	bool simReset = false;

};

#endif
