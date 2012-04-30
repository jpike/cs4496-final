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

	// make identity child matrix (makes recursive calls simpler)
	Mat4d childMatrix;
	childMatrix.MakeDiag(1.0);
	
	// calculate entire jacobian
	CalculateJacobian(jacobianMatrix, node, localPos, childMatrix);

	return jacobianMatrix;
}

///-------------------------------------------------------------
/// Calculates and fills Jacobian matrix for a given 
/// transform node.
///-------------------------------------------------------------
void ConstraintJacobian::CalculateJacobian(Matd & jacobianMatrix, TransformNode * node, Vec4d & localPos, Mat4d & childTransform)
{
	// pre-calculate some values we need later during computation
	// we need matrices within this node's transform that occur
	// before/after a transform with a given DOF
	CalculatePreMatrices(node);
	CalculatePostMatrices(node);

	// get data we need
	Mat4d parentTransform = node->mParentTransform;

	// set matrix for new child transform
	Mat4d newChildTransform;
	newChildTransform.MakeDiag(1.0);

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
				Vec4d value = parentTransform * preMatrix * deriv * postMatrix * childTransform * localPos;

				jacobianMatrix[dofId] = value;
			}
		}
		// add this transform to appropriate new child transformation matrix
		newChildTransform *= transform->GetTransform();
	}

	// calculate jacobian values for parent's degrees of freedom if we have a parent
	TransformNode * parent = node->mParentNode;
	if (parent != NULL && parent != node)	// parent not NULL or current node
	{
		// calculate final new child transform - append current node's transformations to front of old child transform
		newChildTransform *= childTransform;
		CalculateJacobian(jacobianMatrix, parent, localPos, newChildTransform);
	}
}

///-------------------------------------------------------------
/// Calculates matrix that occurs before (left-side)
/// a transform matrix with a given DOF.
///-------------------------------------------------------------
void ConstraintJacobian::CalculatePreMatrices(TransformNode * node)
{
	// clear any old data
	mPreMatrices.clear();

	// get initial data we need
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
		preMatrix *= transform->GetTransform();
	}
}

///-------------------------------------------------------------
/// Calculates matrix that occurs after (right-side)
/// a transform matrix with a given DOF.
///-------------------------------------------------------------
void ConstraintJacobian::CalculatePostMatrices(TransformNode * node)
{
	// clear any old data
	mPostMatrices.clear();

	// get initial data we need
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