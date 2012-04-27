///*************************************************************
/// @file IKSolver.cpp
/// Class to handle most inverse kinematics solving
/// functionality.
///
///*************************************************************

///-------------------------------------------------------------
/// Includes
///-------------------------------------------------------------
#include "IKSolver.h"
#include <fstream>
#include <cfloat>
#include "Marker.h"
#include "TransformNode.h"
#include "Transform.h"
#include "C3dFileInfo.h"

///-------------------------------------------------------------
/// Constructor.
/// Sets parameters for solver.
///-------------------------------------------------------------
IKSolver::IKSolver(double epsilon, double stepSize, int maxIterations, Model * model)
	: mEpsilon(epsilon), mStepSize(stepSize), mMaxIterations(maxIterations), mModel(model)
{
	
}

///-------------------------------------------------------------
/// Destructor.
/// Sets selected model to NULL.
///-------------------------------------------------------------
IKSolver::~IKSolver()
{
	mModel = NULL;
}

///-------------------------------------------------------------
/// Initialization routine.
/// Does some pre-computation and other setup before
/// entering main computation loop.
///-------------------------------------------------------------
void IKSolver::Initialize()
{
	CreateConstraints();
}

///-------------------------------------------------------------
/// Creates a map from dof Id's to handles.
/// This is helpful in figuring out where in the constraint
/// vector a specific value should go.
///-------------------------------------------------------------
void IKSolver::CreateDofToHandleMap()
{
	// clear of any old values
	mDofToHandleMap.clear();
	mHandleToDofMap.clear();

	// Loop through all handles
	std::vector<Marker *> handleList = mModel->mHandleList;
	for (int i = 0; i < handleList.size(); i++)
	{
		// Get marker
		Marker * marker = handleList[i];

		// Get transform node (used to get dofs)
		TransformNode * transformNode = mModel->mLimbs[marker->mNodeIndex];

		// get actual transforms in node
		std::vector<Transform *> transforms = transformNode->mTransforms;

		// loop over all transforms
		for (int j = 0; j < transforms.size(); j++)
		{
			Transform * transform = transforms[j];

			// if we have a transform with DOFs, we need to get those DOFs
			if (transform->IsDof())
			{
				int numDofs = transform->GetDofCount();
				// loop over all DOFs to add appropriate handle to our map
				for (int k = 0; k < numDofs; k++)
				{
					// get dof
					Dof * dof = transform->GetDof(k);
					int dofId = dof->mId;

					// add to maps
					mDofToHandleMap[dofId] = marker;

					mHandleToDofMap.insert(std::pair<Marker *, int>(marker, dofId));
				}
			}
		}
	}

#ifdef _DEBUG
	LogDofToHandleMap();
#endif
}

///-------------------------------------------------------------
/// Prints dof to handle map to log file.
///-------------------------------------------------------------
void IKSolver::LogDofToHandleMap()
{
	std::ofstream logFile("logs/dof_to_handle.txt");	
	
	// loop and print data for first map
	DofIdToMarkerMap::iterator iter;
	for (iter = mDofToHandleMap.begin(); iter != mDofToHandleMap.end(); iter++)
	{
		// get data
		int dofId = iter->first;
		Marker * marker = iter->second;

		// print data
		logFile << "Dof " << dofId << "\tMarker " << marker->mMarkerOrder << std::endl;
	}

	// loop and print data for second map
	MarkerToDofIdMap::iterator iter2;
	for (iter2 = mHandleToDofMap.begin(); iter2 != mHandleToDofMap.end(); iter2++)
	{
		// get data
		Marker * marker = iter2->first;
		int dofId = iter2->second;

		// print data
		logFile << "Handle " << marker->mMarkerOrder << "\tDof " << dofId << std::endl;
	}
	
	logFile.close();
}

///-------------------------------------------------------------
/// Main solving loop.
/// Loops over all valid frames and performs IK solving.
///-------------------------------------------------------------
void IKSolver::SolveLoop()
{
	int maxFrames = mModel->mOpenedC3dFile->GetFrameCount();

#ifdef _DEBUG
	std::ofstream logFile("logs/loop_log.txt");

	logFile << "Num Frames: " << maxFrames << std::endl << std::endl;
#endif

	// loop over all valid frames
	// this is limited by the number of frames in the constraint file, but it is also
	// limited by a max iteration parameter set for the solver
	for (int frameCounter = 0; frameCounter < maxFrames; frameCounter++)
	{
		std::cout << "Starting frame " << frameCounter << std::endl;
		// calculate constraint values
		CalculateConstraints(frameCounter);
		// evaluate objective function
		double objectiveFunction = EvaluateObjectiveFunction(frameCounter);

		int iterations = 0;

		while (objectiveFunction > mEpsilon )//&& iterations < mMaxIterations)
		{
			
#ifdef _DEBUG
			logFile << "Frame: " << frameCounter << std::endl;
			logFile << "Iteration: " << iterations << std::endl;
			logFile << "Objective Function: " << objectiveFunction << std::endl;
#endif

			// calculate gradient
			Vecd gradient = CalculateGradient(frameCounter);

			// get old dofs
			Vecd oldDofs;
			oldDofs.SetSize(mModel->GetDofCount());
			mModel->mDofList.GetDofs(&oldDofs);

			// move dofs
			Vecd newDofs = oldDofs - mStepSize * gradient;

			// update dofs
			mModel->SetDofs(newDofs);

#ifdef _DEBUG
			logFile << "Gradient: " << gradient << std::endl;
			logFile << "Old Dofs: " << oldDofs << std::endl;
			logFile << "New Dofs: " << newDofs << std::endl;
			logFile << std::endl;
#endif

			// calculate new constraint values
			CalculateConstraints(frameCounter);
			// calculate new objective function value
			objectiveFunction = EvaluateObjectiveFunction(frameCounter);

			// update iteration counter
			iterations++;
		}

	}

#ifdef _DEBUG
	logFile.close();
#endif
}

///-------------------------------------------------------------
/// Creates initial list of constraints.
///-------------------------------------------------------------
void IKSolver::CreateConstraints()
{
	// clear any old data that might exist
	mConstraintList.clear();

	// get all handles on model
	MarkerList & modelHandles = mModel->mHandleList;

	// loop over all handles on model, evaluating constraint
	for (int i = 0; i < modelHandles.size(); i++)
	{
		// get handle on model and contraint position
		Marker * handle = modelHandles[i];
		Vec3d & constraintPos = mModel->mOpenedC3dFile->GetMarkerPos(0, i);

		// if constraint is 0, 0, 0, then we don't have a constraint to actually deal with,
		// so only create and add constraint if this is not the case
		if (constraintPos != vl_zero)
		{
			mConstraintList.push_back(Constraint(mModel, i));
		}
	}

#ifdef _DEBUG
	LogConstraintList(0, false);
#endif
}

///-------------------------------------------------------------
/// Calculates constraints for a given frame.
///-------------------------------------------------------------
void IKSolver::CalculateConstraints(int frameNum)
{
	// loop over all constraints, updating values
	for (int i = 0; i < mConstraintList.size(); i++)
	{
		Constraint & constraint = mConstraintList[i];
		constraint.EvaluateConstraint(frameNum);
	}

#ifdef _DEBUG
	LogConstraintList(frameNum, true);
#endif
}

///-------------------------------------------------------------
/// Logs constraint list.
///-------------------------------------------------------------
void IKSolver::LogConstraintList(int frameNum, bool append)
{
	std::ios::openmode openMode = std::ios::out;
	if (append)
	{
		openMode = std::ios::app;
	}
	std::ofstream logFile("logs/constraints.txt", openMode);

	logFile << "Frame: " << frameNum << std::endl;
	logFile << "Num Constraints: " << mConstraintList.size() << std::endl;
	// loop and print data
	for (int i = 0; i < mConstraintList.size(); i++)
	{
		// get data
		Constraint & constraint = mConstraintList[i];

		// log data
		logFile << "Constraint " << i << "\tId: " << constraint.GetConstraintId() 
				<< "\tHandle Pos: " << constraint.GetHandleGlobalPos()
				<< "\tConstraint Pos: " << constraint.GetConstraintPos(frameNum)
				<< "\tValue: " << constraint.GetConstraintValue()
				<< "\tSqrLen: " << constraint.GetConstraintSquareLength()
				<< std::endl;
	}
	logFile << std::endl;

	logFile.close();
}

///-------------------------------------------------------------
/// Calculates gradient for a given frame.
/// TODO
///-------------------------------------------------------------
Vecd IKSolver::CalculateGradient(int frameNum)
{
	// setup initial gradient to be zero
	// the dimensions should be equal to the number of DOFs we have
	Vecd gradient;
	gradient.SetSize(mModel->GetDofCount());
	gradient.MakeZero();

	// loop over all of our constraints, getting Jacobian and
	// combining to form final value to add to total gradient
	for (int i = 0; i < mConstraintList.size(); i++)
	{
		// get constraint
		Constraint & constraint = mConstraintList[i];
		Vec4d constraintVec(constraint.GetConstraintValue(), 1.0);

		// get Jacobian - TODO
		ConstraintJacobian jacobian(mModel, constraint);
		Matd jacobianMatrix = jacobian.CalculateJacobian();		

		// calculate current value and add to gradient
		Vecd currentValue = jacobianMatrix * constraintVec;
		gradient = gradient + currentValue;
	}

	// final computation of doubling after above calculation
	gradient = 2.0 * gradient;

	return gradient;
}

///-------------------------------------------------------------
/// Evaluates objective function for a given frame.
/// TODO
///-------------------------------------------------------------
double IKSolver::EvaluateObjectiveFunction(int frameNum)
{
	double objectiveFunction = 0.0;

	// loop over all constraints, adding to value
	for (int i = 0; i < mConstraintList.size(); i++)
	{
		// get constraint
		Constraint & constraint = mConstraintList[i];

		// add sqrlen value to function value
		objectiveFunction += constraint.GetConstraintSquareLength();
	}

	return objectiveFunction;
}

///-------------------------------------------------------------
/// Calculates constraint map for a given frame.
///-------------------------------------------------------------
void IKSolver::CreateConstraintMap(int frameNum)
{
	// clear any old data (from previous frames)
	mHandleToConstraintMap.clear();

	// get all handles on model
	MarkerList & modelHandles = mModel->mHandleList;

	// loop over all handles on model, evaluating constraint
	for (int i = 0; i < modelHandles.size(); i++)
	{
		// get handle on model and contraint position
		Marker * handle = modelHandles[i];
		Vec3d & constraintPos = mModel->mOpenedC3dFile->GetMarkerPos(frameNum, i);

		// if constraint is 0, 0, 0, then we don't have a constraint to actually deal with
		if (constraintPos == vl_zero)
		{
			// TODO: MIGHT NEED TO MAKE THIS VALUE HAVE A 1 IN W POSITION INSTEAD OF 0	
			mHandleToConstraintMap[handle] = vl_zero;	// put zero for constraint value (no movement for marker)
			continue;
		}

		// evaluate constraint function
		Vec3d constraint = EvaluateConstraint(handle, constraintPos);

		// add to constraint vector
		mHandleToConstraintMap[handle] = Vec4d(constraint, 1.0);
	}

#ifdef _DEBUG
	LogConstraintMap();
#endif
}

///-------------------------------------------------------------
/// Logs constraint map for a given frame.
///-------------------------------------------------------------
void IKSolver::LogConstraintMap()
{
	std::ofstream logFile("logs/handle_to_constraint.txt");

	// loop and print data
	MarkerToPosMap::iterator iter;
	for (iter = mHandleToConstraintMap.begin(); iter != mHandleToConstraintMap.end(); iter++)
	{
		// get data
		int handleId = iter->first->mMarkerOrder;
		Vec4d constraint = iter->second;

		// log data
		logFile << "Handle " << handleId << "\tConstraint " << constraint << std::endl;
	}

	logFile.close();
}

//----------------------------------------------------------------------
// Evaluates constraint for a given handle and target position.
//----------------------------------------------------------------------
Vec3d IKSolver::EvaluateConstraint(Marker * handle, Vec3d & constraintPos)
{
	// returns vector from desired position to current position
	// can be used to get distance between 2 positions
	return (handle->mGlobalPos - constraintPos);
}

//----------------------------------------------------------------------
// Converts map of constraints to appropriate matrix form
// for utlilizing multiplication of VL library.
//----------------------------------------------------------------------
void IKSolver::CreateConstraintMatrix()
{
	// get data needed for matrix size
	int numDofs = mModel->GetDofCount();

	// set matrix size
	// 4 is for xyzw components of Vec4d
	mConstraintMatrix.SetSize(numDofs, 4);

	// clear old matrix data
	mConstraintMatrix.MakeZero();	// TODO: MIGHT NEED TO MAKE COLUMN 4 IN MATRIX BE 1's INSTEAD OF 0's	

	// loop over constraint map to get constraints
	MarkerToPosMap::iterator iter;
	for (iter = mHandleToConstraintMap.begin(); iter != mHandleToConstraintMap.end(); iter++)
	{
		// get data
		Marker * handle = iter->first;
		int handleId = handle->mMarkerOrder;
		Vec4d constraint = iter->second;

		// get dofs we need to set constraint for
		// we loop over all values (DOF Ids) in map that are part of this handle
		// and set constraint to appropriate position in matrix
		std::pair< MarkerToDofIdMap::iterator, MarkerToDofIdMap::iterator > range = mHandleToDofMap.equal_range(handle);
		MarkerToDofIdMap::iterator iter2;
		for (iter2 = range.first; iter2 != range.second; iter2++)
		{
			// just some error checking with marker
			// these values should be the same since we should be getting values with this same handle key
			Marker * marker = iter2->first;
			if (handle != marker)
			{
				std::cerr << "Error!  Handle does not equal marker! (IKSolver::CreateConstraintMatrix())" << std::endl;
			}

			// get dofId (index into constraint matrix)
			int dofId = iter2->second;

			// set constraint value for this dof
			mConstraintMatrix[dofId] = constraint;
		}
	}

#ifdef _DEBUG
	LogConstraintMatrix();
#endif
}

///-------------------------------------------------------------
/// Logs constraint matrix.
///-------------------------------------------------------------
void IKSolver::LogConstraintMatrix()
{
	std::ofstream logFile("logs/constraint_matrix.txt");

	logFile << "Constraint Matrix: " << std::endl;
	logFile << mConstraintMatrix << std::endl;

	logFile.close();
}

///-------------------------------------------------------------
/// Computes left-hand matrix on side of a dof
/// for forming Jacobian.
///-------------------------------------------------------------
void IKSolver::CreatePreMatrices()
{
	// clear any old data
	mDofToPreMatrixMap.clear();

	// loop over all dofs (1 computation per DOF)
	DofIdToMarkerMap::iterator iter;
	for (iter = mDofToHandleMap.begin(); iter != mDofToHandleMap.end(); iter++)
	{
		// get basic data
		int dofId = iter->first;
		Marker * handle = iter->second;

		// get node with transform data
		TransformNode * transformNode = mModel->mLimbs[handle->mNodeIndex];
		// get parent transform
		Mat4d parentTransform = transformNode->mParentTransform;
		// form initial pre/left side matrix
		Mat4d preMatrix = vl_I * parentTransform;

		// loop over transforms (and DOFs) within this transform node
		// until we reach a dof with same id as the one we are currently examining
		std::vector<Transform *> transforms = transformNode->mTransforms;
		// if true, then we've reached matrix with this DOF in chain of transforms, so we can stop looping over transforms
		bool dofReached = false;	
		for (int i = 0; i < transforms.size(); i++)
		{
			Transform * transform = transforms[i];
			// if not a DOF, then just append to preMatrix
			if (!transform->IsDof())
			{
				Mat4d matrix = transform->GetTransform();
				preMatrix = preMatrix * matrix;
			} // end non-DOF transform case
			// if it is a DOF transform, then we need to loop over some DOFs
			else
			{
				int dofCount = transform->GetDofCount();
				for (int j = 0; j < dofCount; j++)
				{
					Dof * dof = transform->GetDof(j);
					int transformDofId = dof->mId;
					// if we have matching DOF ids, then we've computed all 
					// matrices on left-hand side of this DOF, so we need to stop 
					// looping over DOFs and transforms
					if (dofId == transformDofId)
					{
						dofReached = true;
						break;
					}
				}	// end loop over DOFs in DOF transform
				// if DOF was not reached, then we can append this matrix to preMatrix
				if (!dofReached)
				{
					preMatrix = preMatrix * transform->GetTransform();
				}
			}	// end case for DOF transform
			// if DOF was reached, we need to stop going through transforms
			if (dofReached)
			{
				break;
			}
		} // end looping over transforms

		// once we finish loop, we've computed the appropriate left hand side,
		// so add it to appropriate spot in map
		mDofToPreMatrixMap[dofId] = preMatrix;

	}	// end looping over DOFs

#ifdef _DEBUG
	LogPreMatrices();
#endif
}	// end computing pre matrices

///-------------------------------------------------------------
/// Logs pre-matrices.
///-------------------------------------------------------------
void IKSolver::LogPreMatrices()
{
	std::ofstream logFile("logs/pre_matrix.txt");

	logFile << "Pre Matrices: " << std::endl;

	// loop over all entries in map
	DofIdToMatrixMap::iterator iter;
	for (iter = mDofToPreMatrixMap.begin(); iter != mDofToPreMatrixMap.end(); iter++)
	{
		// get data
		int dofId = iter->first;
		Mat4d matrix = iter->second;

		// log data
		logFile << "Dof " << dofId << std::endl;
		logFile << matrix << std::endl;
	}

	logFile.close();
}

///-------------------------------------------------------------
/// Computes right-hand matrix on side of a dof
/// for forming Jacobian.
///-------------------------------------------------------------
void IKSolver::CreatePostMatrices()
{
	// clear any old data
	mDofToPostMatrixMap.clear();

	// loop over all dofs (1 computation per DOF)
	DofIdToMarkerMap::iterator iter;
	for (iter = mDofToHandleMap.begin(); iter != mDofToHandleMap.end(); iter++)
	{
		// get basic data
		int dofId = iter->first;
		Marker * handle = iter->second;

		// get node with transform data
		TransformNode * transformNode = mModel->mLimbs[handle->mNodeIndex];
		// form initial post/right side matrix
		Mat4d postMatrix = vl_I;

		// loop over transforms (and DOFs) within this transform node
		// until we reach a dof with same id as the one we are currently examining
		// once this happens, we can start adding to our postMatrix
		std::vector<Transform *> transforms = transformNode->mTransforms;
		// if true, then we've reached matrix with this DOF in chain of transforms, so we can start updating postMatrix
		bool dofReached = false;	
		for (int i = 0; i < transforms.size(); i++)
		{
			Transform * transform = transforms[i];

			// if dof has been reached, then continue appending to end of postMatrix
			if (dofReached)
			{
				Mat4d matrix = transform->GetTransform();
				postMatrix = postMatrix * matrix;
			}
			// if dof has not been reached yet, check if current transform
			// has this dof
			else
			{
				if (transform->IsDof())
				{
					int dofCount = transform->GetDofCount();
					for (int j = 0; j < dofCount; j++)
					{
						Dof * dof = transform->GetDof(j);
						int transformDofId = dof->mId;
						// if we have matching DOF ids, then we've finally reached
						// the matrix with this DOF, so we can start append to
						// our postMatrix next iteration
						if (dofId == transformDofId)
						{
							dofReached = true;
							break;
						}
					}	// end loop over DOFs in DOF transform
				} // end case where transform is DOF
			} // end case of dof not yet reached

			
		} // end looping over transforms

		// once we finish loop, we've almost computed the appropriate right hand side.
		// we just need to append the final local coordinates of the handle
		Vec4d localPos(handle->mOffset, 1.0);
		mDofToPostMatrixMap[dofId] = postMatrix * localPos;

	}	// end looping over DOFs

#ifdef _DEBUG
	LogPostMatrices();
#endif
}

///-------------------------------------------------------------
/// Logs post-matrices.
///-------------------------------------------------------------
void IKSolver::LogPostMatrices()
{
	std::ofstream logFile("logs/post_matrix.txt");

	logFile << "Post Matrices: " << std::endl;

	// loop over all entries in map
	DofIdToVectorMap::iterator iter;
	for (iter = mDofToPostMatrixMap.begin(); iter != mDofToPostMatrixMap.end(); iter++)
	{
		// get data
		int dofId = iter->first;
		Vec4d matrix = iter->second;

		// log data
		logFile << "Dof " << dofId << std::endl;
		logFile << matrix << std::endl;
	}

	logFile.close();
}

///-------------------------------------------------------------
/// Computes derivative matrix of a dof
/// for forming Jacobian.
///-------------------------------------------------------------
void IKSolver::CreateDerivatives()
{
	// clear any old data
	mDofToDerivativeMap.clear();

	// loop over all dofs (1 computation per DOF)
	DofIdToMarkerMap::iterator iter;
	for (iter = mDofToHandleMap.begin(); iter != mDofToHandleMap.end(); iter++)
	{
		// get basic data
		int dofId = iter->first;
		Marker * handle = iter->second;

		// get node with transform data
		TransformNode * transformNode = mModel->mLimbs[handle->mNodeIndex];

		// get and loop over all transforms to find one with appropriate DOF
		std::vector<Transform *> transforms = transformNode->mTransforms;
		for (int i = 0; i < transforms.size(); i++)
		{
			Transform * transform = transforms[i];

			// only need to check if transform is DOF
			if (transform->IsDof())
			{
				// loop over dofs to see if this transform has matching dof
				int dofCount = transform->GetDofCount();
				for (int j = 0; j < dofCount; j++)
				{
					Dof * dof = transform->GetDof(j);
					int transformDofId = dof->mId;
					// if we find appropriate matching dof,
					// compute derivative and set appropriate value in map
					if (dofId == transformDofId)
					{
						Mat4d deriv = transform->GetDeriv(j);
						mDofToDerivativeMap[dofId] = deriv;
						break;
					}
				}	// end looping over dofs in transform
			} // end case where transform is dof
		}	// end looping over transforms

	}	// end looping over dofs

#ifdef _DEBUG
	LogDerivatives();
#endif
}

///-------------------------------------------------------------
/// Logs derivatives.
///-------------------------------------------------------------
void IKSolver::LogDerivatives()
{
	std::ofstream logFile("logs/derivative_matrix.txt");

	logFile << "Derivative Matrices: " << std::endl;

	// loop over all entries in map
	DofIdToMatrixMap::iterator iter;
	for (iter = mDofToDerivativeMap.begin(); iter != mDofToDerivativeMap.end(); iter++)
	{
		// get data
		int dofId = iter->first;
		Mat4d matrix = iter->second;

		// log data
		logFile << "Dof " << dofId << std::endl;
		logFile << matrix << std::endl;
	}

	logFile.close();
}