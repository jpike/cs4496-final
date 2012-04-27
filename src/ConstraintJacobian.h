#ifndef CONSTRAINT_JACOBIAN_H
#define CONSTRAINT_JACOBIAN_H

///*************************************************************
/// @file ConstraintJacobian.h
/// Class to calculate Jacobian for a given constraint.
///
///*************************************************************

///-------------------------------------------------------------
/// Includes
///-------------------------------------------------------------
#include <map>
#include "Constraint.h"

///-------------------------------------------------------------
/// Typedefs
///-------------------------------------------------------------
typedef std::map<int, Mat4d> DofIdToMatrixMap;

///-------------------------------------------------------------
/// @ConstraintJacobian
/// Class to calculate Jacobian for a given constraint.
///-------------------------------------------------------------
class ConstraintJacobian
{
public:
	ConstraintJacobian(Model * model, Constraint & constraint);
	~ConstraintJacobian();

	Matd CalculateJacobian();

protected:

	// helper methods
	void CalculatePreMatrices();
	void CalculatePostMatrices();

	// helper containers related to helper methods above
	DofIdToMatrixMap mPreMatrices;
	DofIdToMatrixMap mPostMatrices;

	// main jacobian parameters
	Model * mModel;
	Constraint & mConstraint;
};

#endif