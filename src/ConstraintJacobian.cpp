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
	// pre-calculate some values we need later during computation
	// we need matrices within this handle's transform that occur
	// before/after a transform with a given DOF
	CalculatePreMatrices();
	CalculatePostMatrices();

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

				// get row to assign to in jacobian matrix
				Dof * dof = transform->GetDof(j);
				int dofId = dof->mId;

				// TODO: Multiply by parent, prev/post matrices, and localPos
				// and assign to appropriate row in Jacobian
				Mat4d preMatrix = mPreMatrices[dofId];
				Mat4d postMatrix = mPostMatrices[dofId];
				Vec4d value = parentTransform * preMatrix * deriv * postMatrix * localPos;

				jacobianMatrix[dofId] = value;
			}
		}
	}

	return jacobianMatrix;
}

///-------------------------------------------------------------
/// Calculates matrix that occurs before (left-side)
/// a transform matrix with a given DOF.
/// TODO
///-------------------------------------------------------------
void ConstraintJacobian::CalculatePreMatrices()
{
	// clear any old data
	mPreMatrices.clear();

	// get initial data we need
	Marker * handle = mConstraint.GetHandle();
	TransformNode * node = mModel->mLimbs[handle->mNodeIndex];
	std::vector<Transform *> transforms = node->mTransforms;

	// loop over all transforms in node, figuring out
	// appropriate pre-matrix for each DOF transform
	Mat4d preMatrix;
	preMatrix.MakeDiag(1.0);	// start off as identity
	for (int i = 0; i < transforms.size(); i++)
	{
		Transform * transform = transforms[i];
		// only need to calculate if DOF transform
		if (transform->IsDof())
		{
			// loop through each dof
			for (int j = 0; j < transform->GetDofCount(); j++)
			{
				// get dof
				Dof * dof = transform->GetDof(j);
				int dofId = dof->mId;

				// place in appropriate spot in map
				mPreMatrices[dofId] = preMatrix;
			}
		}
		// now that we've gone through this transform, 
		// update the preMatrix to have current transform for later transforms
		preMatrix = preMatrix * transform->GetTransform();
	}
}

///-------------------------------------------------------------
/// Calculates matrix that occurs after (right-side)
/// a transform matrix with a given DOF.
/// TODO
///-------------------------------------------------------------
void ConstraintJacobian::CalculatePostMatrices()
{
	// clear any old data
	mPostMatrices.clear();

	// get initial data we need
	Marker * handle = mConstraint.GetHandle();
	TransformNode * node = mModel->mLimbs[handle->mNodeIndex];
	std::vector<Transform *> transforms = node->mTransforms;

	// loop over all transforms in node, figuring out
	// appropriate post-matrix for each DOF transform
	Mat4d postMatrix;
	postMatrix.MakeDiag(1.0);	// start off as identity
	for (int i = transforms.size() - 1; i >= 0; i--)
	{
		Transform * transform = transforms[i];
		// only need to calculate if DOF transform
		if (transform->IsDof())
		{
			// loop through each dof
			for (int j = transform->GetDofCount() - 1; j >= 0 ; j--)
			{
				// get dof
				Dof * dof = transform->GetDof(j);
				int dofId = dof->mId;

				// place in appropriate spot in map
				mPostMatrices[dofId] = postMatrix;
			}
		}
		// now that we've gone through this transform, 
		// update the postMatrix to have current transform for later transforms
		postMatrix = transform->GetTransform() * postMatrix;
	}
}