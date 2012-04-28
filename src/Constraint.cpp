///*************************************************************
/// @file Constraint.cpp
/// Class representing a constraint function in IK system
///
///*************************************************************

///-------------------------------------------------------------
/// Includes
///-------------------------------------------------------------
#include "Constraint.h"
#include "Marker.h"
#include "C3dFileInfo.h"

///-------------------------------------------------------------
/// Constructor.
/// Sets basic data for constraint.
///-------------------------------------------------------------
Constraint::Constraint(Model * model, int constraintId)
	: mModel(model), mConstraintId(constraintId)
{

}

///-------------------------------------------------------------
/// Destructor.
/// Sets selected model to NULL.
///-------------------------------------------------------------
Constraint::~Constraint()
{
	mModel = NULL;
}

///-------------------------------------------------------------
/// Updates data by evaluating constraint for given frame.
/// Constraint evaluation is current handle distance from
/// target position.
/// Assumes that constraint position is valid (ie. not 0,0,0).
///-------------------------------------------------------------
void Constraint::EvaluateConstraint(int frameNum)
{
	// get all handles on model
	MarkerList & modelHandles = mModel->mHandleList;

	// get handle and target position for this constraint
	Marker * handle = modelHandles[mConstraintId];
	Vec3d constraintPos = mModel->mOpenedC3dFile->GetMarkerPos(frameNum, mConstraintId);

	// calculate values
	mConstraintValue = EvaluateConstraint(handle, constraintPos);
	mConstraintSquareLength = sqrlen(mConstraintValue);

	/*if (constraintPos == vl_zero)
	{
		mConstraintValue.MakeZero();
		mConstraintSquareLength = 0;
	}*/
}

///-------------------------------------------------------------
/// Helper function.
/// Evaluates constraint for a given handle and target position.
///-------------------------------------------------------------
Vec3d Constraint::EvaluateConstraint(Marker * handle, Vec3d & constraintPos)
{
	// returns vector from desired position to current position
	// can be used to get distance between 2 positions
	return (handle->mGlobalPos - constraintPos);
}

///-------------------------------------------------------------
/// Returns most recent vector computed by evaluating 
/// constraint.
///-------------------------------------------------------------	
Vec3d Constraint::GetConstraintValue()
{
	return mConstraintValue;
}

///-------------------------------------------------------------
/// Returns most recent squared length of vector computed
/// by evaluating constraint.
///-------------------------------------------------------------
double Constraint::GetConstraintSquareLength()
{
	return mConstraintSquareLength;
}

///-------------------------------------------------------------
/// Returns handle for this constraint.
///-------------------------------------------------------------
Marker * Constraint::GetHandle()
{
	// get all handles on model
	MarkerList & modelHandles = mModel->mHandleList;

	// get handle for this constraint
	Marker * handle = modelHandles[mConstraintId];

	return handle;
}

///-------------------------------------------------------------
/// Returns global position for this constraint's handle.
///-------------------------------------------------------------
Vec3d Constraint::GetHandleGlobalPos()
{
	Marker * handle = GetHandle();

	return handle->mGlobalPos;
}

///-------------------------------------------------------------
/// Returns targer position for this constraint.
///-------------------------------------------------------------
Vec3d Constraint::GetConstraintPos(int frameNum)
{
	Vec3d constraintPos = mModel->mOpenedC3dFile->GetMarkerPos(frameNum, mConstraintId);

	return constraintPos;
}