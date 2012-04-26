#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <fstream>
#include "vl/VLd.h"
#include <vector>
#include <map>
#include "Marker.h"

typedef void (*Command)(void*);

void LoadModel(void*);
void LoadC3d(void*);
void Exit(void*);
void Solution(void*);

//----------------------------------------------------------------------
// Typedefs
//----------------------------------------------------------------------
//typedef std::map<int, Marker *> DofIdToMarkerMap;

//----------------------------------------------------------------------
// Class to hold some stuff related to IK solver
// Might refactor to other class, etc. later
//----------------------------------------------------------------------
class IK_Solver
{
public:
	// Functions
	static double EvaluateObjectiveFunction(int frameNum, Matd & constraintVector);
	static Vec3d EvaluateConstraint(Marker * handle, Vec3d & constraintPos);

	static void CreateConstraintVector(int frameNum, Matd & constraintVector);

	static void CreateJacobian(int frameNum);

	static void CreateMatrixDerivatives(std::vector<Mat4d> & derivatives);
	
	static void PrintDofs(int frameNum);

	// Variables
	//static DofIdToMarkerMap dofIdToMarkerMap;
};

#endif

