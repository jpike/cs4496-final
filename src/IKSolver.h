#ifndef IK_SOLVER_H
#define IK_SOLVE_H

///*************************************************************
/// @file IKSolver.h
/// Class to handle most inverse kinematics solving
/// functionality.
///
///*************************************************************

///-------------------------------------------------------------
/// Includes
///-------------------------------------------------------------
#include <map>
#include <vector>
#include "Model.h"
#include "Constraint.h"
#include "ConstraintJacobian.h"

///-------------------------------------------------------------
/// Typedefs
///-------------------------------------------------------------
typedef std::map<int, Marker *> DofIdToMarkerMap;
typedef std::multimap<Marker *, int> MarkerToDofIdMap;
typedef std::map<Marker *, Vec4d> MarkerToPosMap;
typedef std::map<int, Vec4d> DofIdToVectorMap;
typedef std::vector<Constraint> ConstraintList;

///-------------------------------------------------------------
/// @IKSolver
/// Class to handle most inverse kinematics solving
/// functionality.
///-------------------------------------------------------------
class IKSolver
{
public:
	IKSolver(double epsilon, double stepSize, int maxIterations, int maxFrames, int printFrequency, 
			int stepIncreaseFrequency, double stepIncreaseFactor, double stepDecreaseFactor, 
			int epsilonIncreaseFrequency, double epsilonIncreaseFactor, Model * model);
	~IKSolver();

	void Initialize();
	void SolveLoop();

protected:

	// initialization helper functions
	void CreateConstraints(int frameNum);
	void LogConstraintList(int frameNum, bool append);

	// loop-solving helper functions
	void CalculateConstraints(int frameNum);
	double EvaluateObjectiveFunction(int frameNum);
	Vecd CalculateGradient(int frameNum);

	// save dofs for later playback
	void SaveDofs(int frameNum);

	// parameters for an individual "solving"
	double mEpsilon;
	double mStepSize;
	int mMaxIterations;
	int mMaxNumFrames;
	int mPrintFrequency;
	int mStepIncreaseFrequency;
	double mStepIncreaseFactor;
	double mStepDecreaseFactor;
	int mEpsilonIncreaseFrequency;
	double mEpsilonIncreaseFactor;
	Model * mModel;

	// data needed for computation
	ConstraintList mConstraintList;

};

#endif