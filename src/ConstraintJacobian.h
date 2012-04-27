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
#include "Constraint.h"

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

	Model * mModel;
	Constraint & mConstraint;

};

#endif