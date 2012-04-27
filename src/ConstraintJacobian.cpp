///*************************************************************
/// @file ConstraintJacobian.cpp
/// Class to calculate Jacobian for a given constraint.
///
///*************************************************************

///-------------------------------------------------------------
/// Includes
///-------------------------------------------------------------
#include "ConstraintJacobian.h"
#include "Marker.h"
#include "TransformNode.h"
#include "Transform.h"

///-------------------------------------------------------------
/// Constructor.
/// Sets basic data for ibject.
///-------------------------------------------------------------
ConstraintJacobian::ConstraintJacobian(Model * model, Constraint & constraint)
	: mModel(model), mConstraint(constraint)
{

}

///-------------------------------------------------------------
/// Destructor.
/// Sets selected model to NULL.
///-------------------------------------------------------------
ConstraintJacobian::~ConstraintJacobian()
{
	mModel = NULL;
}

///-------------------------------------------------------------
/// Calculates and returns Jacobian matrix (actually transpose).
/// Returned matrix should have dimensions of 
/// #dofs (rows) * 4 (cols)
///-------------------------------------------------------------
Matd ConstraintJacobian::CalculateJacobian()
{
	// create initial jacobian with zeroes
	Matd jacobianMatrix;
	jacobianMatrix.SetSize(mModel->GetDofCount(), 4);
	jacobianMatrix.MakeZero();

	// get initial data we need to compute jacobian for this constraint
	Marker * handle = mConstraint.GetHandle();
	Vec4d localPos(handle->mOffset, 1.0);
	TransformNode * node = mModel->mLimbs[handle->mNodeIndex];
	Mat4d parentTransform = node->mParentTransform;

	// fill in matrix row, by row
	// we do this (instead of column by column) as it is easier
	// to assign to rows and we have to take transpose anyway
	//
	// we need to loop through all transforms, calculating
	// derivatives for those transforms with DOFs
	std::vector<Transform *> transforms = node->mTransforms;
	for (int i = 0; i < transforms.size(); i++)
	{
		// TODO: Get transforms before/after this transform
		
		// check if current transform is DOF; if so, compute derivatives
		Transform * transform = transforms[i];
		if (transform->IsDof())
		{
			// loop through all dofs in transform
			for (int j = 0; j < transform->GetDofCount(); j++)
			{
				Mat4d deriv = transform->GetDeriv(j);
				// TODO: Multiply by parent, prev/post matrices, and localPos
				// and assign to appropriate row in Jacobian

				// get row to assign to in jacobian matrix
				Dof * dof = transform->GetDof(j);
				int dofId = dof->mId;
			}
		}
	}

	return jacobianMatrix;
}