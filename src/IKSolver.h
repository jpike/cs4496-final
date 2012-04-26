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
#include "Model.h"

///-------------------------------------------------------------
/// Typedefs
///-------------------------------------------------------------
typedef std::map<int, Marker *> DofIdToMarkerMap;
typedef std::multimap<Marker *, int> MarkerToDofIdMap;
typedef std::map<Marker *, Vec4d> MarkerToPosMap;
typedef std::vector<Marker*> MarkerList;
typedef std::map<int, Mat4d> DofIdToMatrixMap;
typedef std::map<int, Vec4d> DofIdToVectorMap;

///-------------------------------------------------------------
/// @IKSolver
/// Class to handle most inverse kinematics solving
/// functionality.
///-------------------------------------------------------------
class IKSolver
{
public:
	IKSolver(double epsilon, int maxIterations, Model * model);
	~IKSolver();

	void Initialize();
	void SolveLoop();

protected:

	// initialization helper functions
	void CreateDofToHandleMap();
	void LogDofToHandleMap();

	// loop-solving helper functions
	void CreateConstraintMap(int frameNum);
	void LogConstraintMap();
	Vec3d EvaluateConstraint(Marker * handle, Vec3d & constraintPos);
	void CreateConstraintMatrix();
	void LogConstraintMatrix();
	void CreatePreMatrices();
	void LogPreMatrices();
	void CreatePostMatrices();
	void LogPostMatrices();
	void CreateDerivatives();
	void LogDerivatives();

	// parameters for an individual "solving"
	double mEpsilon;
	int mMaxIterations;
	Model * mModel;

	// maps for easier access to certain data
	DofIdToMarkerMap mDofToHandleMap;
	MarkerToDofIdMap mHandleToDofMap;
	MarkerToPosMap mHandleToConstraintMap;
	DofIdToMatrixMap mDofToPreMatrixMap;
	DofIdToVectorMap mDofToPostMatrixMap;
	DofIdToMatrixMap mDofToDerivativeMap;

	// math versions of data used in computation
	Matd mConstraintMatrix;
};

#endif