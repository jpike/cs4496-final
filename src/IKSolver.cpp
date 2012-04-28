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
#include "RealTimeIKui.h"
#include "PhylterGLWindow.h"

///-------------------------------------------------------------
/// Externs
///-------------------------------------------------------------
extern RealTimeIKUI *UI;

///-------------------------------------------------------------
/// Constructor.
/// Sets parameters for solver.
///-------------------------------------------------------------
IKSolver::IKSolver(double epsilon, double stepSize, int maxIterations, int maxFrames, Model * model)
	: mEpsilon(epsilon), mStepSize(stepSize), mMaxIterations(maxIterations), mMaxNumFrames(maxFrames), mModel(model)
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
	for (int frameCounter = 0; frameCounter < maxFrames && frameCounter < mMaxNumFrames; frameCounter++)
	{
		std::cout << "Starting frame " << frameCounter << std::endl;
		// calculate constraint values
		CreateConstraints(frameCounter);
		CalculateConstraints(frameCounter);
		// evaluate objective function
		double objectiveFunction = EvaluateObjectiveFunction(frameCounter);

		double localStepSize = mStepSize;
		double localEpsilon = mEpsilon;
		int iterations = 0;

		int decreaseStepCounter = 0;

		while (objectiveFunction > localEpsilon )//&& iterations < mMaxIterations)
		{
			std::cout << "Iteration " << iterations << "\tObjective Function: " << objectiveFunction 
						<< "\tStep Size: " << localStepSize << std::endl;
			
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
			Vecd newDofs = oldDofs - localStepSize * gradient;

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
			double newObjectiveFunction = EvaluateObjectiveFunction(frameCounter);
			// make sure we always decrease so that we don't get stuck in some infinite loop
			if (newObjectiveFunction < objectiveFunction)
			{
				// adaptive step size
				// if difference between objective functions is greater than epsilon
				// and certain number of iterations have passed, increase step size
				if (newObjectiveFunction > mEpsilon &&
					iterations > 0 && iterations % 20 == 0)
				{
					localStepSize *= 4.0;
				}

				objectiveFunction = newObjectiveFunction;
			}
			else
			{
#ifdef _DEBUG
				logFile << "OBJECTIVE FUNCTION INCREASED!\t" << newObjectiveFunction << std::endl;
#endif
				
				// half step size
				decreaseStepCounter++;
				localStepSize /= 2.0;
				
				//localEpsilon *= 2.0;
				std::cout << "OBJECTIVE FUNCTION INCREASED!\t" << newObjectiveFunction << "\tEpsilon " << localEpsilon << std::endl;
				//if (decreaseStepCounter > 10)	// if we reach this pont, just go ahead and stop
				//{
					//objectiveFunction = 0.0;
				//}
				//else
				//{
					if (decreaseStepCounter < 300)
					{
						// reset to old dofs
						mModel->SetDofs(oldDofs);
					}
					else
					{
						// at this point, give up; it isn't worth it
						objectiveFunction = 0.0;
					}
				//}
			}

			// update iteration counter
			iterations++;
		}

		UI->mGLWindow->flush();	// update screen
		std::cout << "Ending frame " << frameCounter << std::endl;

	}

#ifdef _DEBUG
	logFile.close();
#endif
}

///-------------------------------------------------------------
/// Creates initial list of constraints.
///-------------------------------------------------------------
void IKSolver::CreateConstraints(int frameNum)
{
	// clear any old data that might exist
	mConstraintList.clear();

	// get all handles on model
	MarkerList & modelHandles = mModel->mHandleList;

	Vec3d zeroVec(0, 0, 0);
	zeroVec.MakeZero();

	// loop over all handles on model, evaluating constraint
	for (int i = 0; i < modelHandles.size(); i++)
	{
		// get handle on model and contraint position
		Marker * handle = modelHandles[i];
		Vec3d & constraintPos = mModel->mOpenedC3dFile->GetMarkerPos(frameNum, i);

		// if constraint is 0, 0, 0, then we don't have a constraint to actually deal with,
		// so only create and add constraint if this is not the case
		// all of these checks are in place to absolutely make sure, because somehow some of
		// these constraints snuck through
		if (constraintPos != vl_zero && constraintPos != vl_0)
		{
			if (len(constraintPos) != 0)
			{
				if ( !(constraintPos[0] == 0 && constraintPos[1] == 0 && constraintPos[2] == 0) )
				{
					if (zeroVec != constraintPos)
					{
						mConstraintList.push_back(Constraint(mModel, i));
					}
				}
			}

		}
	}

	std::cout << mConstraintList.size() << std::endl;

#ifdef _DEBUG
	LogConstraintList(0, false);
#endif
}

///---------------------------------r----------------------------
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
	if (frameNum > 10 && frameNum < 47) return;

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

