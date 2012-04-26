#ifndef CONSTRAINT_H
#define CONSTRAINT_H

///*************************************************************
/// @file Constraint.h
/// Class representing a constraint function in IK system
///
///*************************************************************

///-------------------------------------------------------------
/// Includes
///-------------------------------------------------------------
#include "Model.h"

///-------------------------------------------------------------
/// @Constraint
/// Class representing a constraint function in IK system
///-------------------------------------------------------------
class Constraint
{
public:
	Constraint(Model * model, int constraintId);
	~Constraint();

	int GetConstraintId()	{ return mConstraintId; }

	// updates data by evaluating constraint for given frame
	void EvaluateConstraint(int frameNum);
	// returns most recent data computed by evaluating constraint
	Vec3d GetConstraintValue();
	double GetConstraintSquareLength();

protected:
	// helper function
	Vec3d EvaluateConstraint(Marker * handle, Vec3d & constraintPos);

	// constraint identifying parameters
	Model * mModel;
	int mConstraintId;

	// data updated during constraint evaluation
	Vec3d mConstraintValue;
	double mConstraintSquareLength;

};

#endif