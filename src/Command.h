#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <fstream>
#include "vl/VLd.h"
#include <vector>
#include "Marker.h"

typedef void (*Command)(void*);

void LoadModel(void*);
void LoadC3d(void*);
void Exit(void*);
void Solution(void*);

//----------------------------------------------------------------------
// Namespace to hold some stuff related to IK solver
// Might refactor to other class, etc. later
//----------------------------------------------------------------------
namespace IK_Solver
{
	double EvaluateObjectiveFunction(int frameNum);
	Vec3d EvaluateConstraint(Marker * handle, Vec3d & constraintPos);
}

#endif

