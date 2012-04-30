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
#include <hash_map>
#include "Constraint.h"
#include "TransformNode.h"

///-------------------------------------------------------------
/// Typedefs
///-------------------------------------------------------------
typedef std::hash_map<int, Mat4d> DofIdToMatrixMap;

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
	void CalculateJacobian(Matd & jacobianMatrix, TransformNode * node, Vec4d & localPos, Mat4d & childTransform);
	void CalculatePreMatrices(TransformNode * node);
	void CalculatePostMatrices(TransformNode * node);

	// helper containers related to helper methods above
	DofIdToMatrixMap mPreMatrices;
	DofIdToMatrixMap mPostMatrices;

	// main jacobian parameters
	Model * mModel;
	Constraint & mConstraint;
};

#endif